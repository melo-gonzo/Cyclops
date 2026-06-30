// web_auth.cpp - see web_auth.h for why this exists.

#include "web_auth.h"
#include "branding.h"
#include "presence.h"  // connected-client tracker (counts distinct recent IPs)
#include "http_parse.h" // pure header/cookie parsing (unit-tested in test/test_http_parse)
#include <MD5Builder.h>
#include <Preferences.h>
#include <time.h>

static const char REALM[] = DEVICE_NAME;
#define NONCE_TTL_MS 43200000UL // 12h, then a silent stale=true re-auth

// "Remember me" session: after one successful digest login the browser gets a
// signed cookie and stops being prompted. This matters most on iOS, where
// WebKit does not persist digest credentials across page loads (so every visit
// re-prompts) but does persist cookies.
#define SESSION_SECS    (30L * 24 * 3600) // cookie lifetime
#define CLOCK_SET_EPOCH 1700000000L       // time(NULL) above this = real clock
static const char COOKIE_NAME[] = "cyc_auth";

static String md5(const String &s) {
  MD5Builder b;
  b.begin();
  b.add(s);
  b.calculate();
  return b.toString();
}

// Web-login credentials live in NVS ("auth"), seeded from the branding defaults
// on first boot and editable at /wifi. Cached in RAM so the per-request auth
// path never hits flash. DEVICE_DEFAULT_PASS is empty by default, so out of the
// box g_pass is "" => passwordless (see webAuthEnabled / webAuthCheck).
static String g_user, g_pass;     // admin (full access)
static String g_vuser, g_vpass;   // read-only viewer ("" user = disabled)
static bool g_loaded = false;

// Roles. g_reqRole is the role the most recent webAuthCheck() authenticated as;
// it's read right after the check on the single-threaded server task.
enum { ROLE_ADMIN = 0, ROLE_VIEWER = 1 };
static int g_reqRole = ROLE_ADMIN;

static void loadCreds() {
  if (g_loaded) return;
  g_loaded = true;
  g_user = DEVICE_DEFAULT_USER;
  g_pass = DEVICE_DEFAULT_PASS;
  Preferences p;
  if (p.begin("auth", true)) {
    g_user  = p.getString("u", g_user);
    g_pass  = p.getString("p", g_pass);
    g_vuser = p.getString("vu", "");   // viewer off until configured
    g_vpass = p.getString("vp", "");
    p.end();
  }
}

String webAuthGetUser() { loadCreds(); return g_user; }
String webAuthGetPass() { loadCreds(); return g_pass; }

// Auth is "on" only when an admin password is set. An empty password means
// passwordless mode: webAuthCheck() admits every request as admin with no
// prompt. This is the factory default and what the /wifi toggle restores.
bool webAuthEnabled() { loadCreds(); return g_pass.length() > 0; }

// Turn auth off (back to passwordless) by clearing the admin password. The
// username is kept, so re-enabling later is just setting a password again.
// (webAuthSetCreds can't express this: it deliberately ignores empty fields so
// the username can be changed without retyping the password.)
void webAuthDisable() {
  loadCreds();
  g_pass = "";
  Preferences p;
  if (p.begin("auth", false)) { p.putString("p", g_pass); p.end(); }
}

void webAuthSetCreds(const String &user, const String &pass) {
  loadCreds();
  if (user.length()) g_user = user;
  if (pass.length()) g_pass = pass;
  Preferences p;
  if (p.begin("auth", false)) {
    p.putString("u", g_user);
    p.putString("p", g_pass);
    p.end();
  }
}

String webAuthGetViewerUser() { loadCreds(); return g_vuser; }
bool   webAuthViewerEnabled() { loadCreds(); return g_vuser.length() > 0; }
bool   webAuthIsViewer()      { return g_reqRole == ROLE_VIEWER; }

// An empty username disables the viewer (and clears its password). Otherwise both
// fields are replaced (a viewer login isn't useful with a blank password, so unlike
// the admin setter we set both together).
void webAuthSetViewerCreds(const String &user, const String &pass) {
  loadCreds();
  g_vuser = user;
  g_vpass = user.length() ? pass : String("");
  Preferences p;
  if (p.begin("auth", false)) {
    p.putString("vu", g_vuser);
    p.putString("vp", g_vpass);
    p.end();
  }
}

// Derived from the credentials, so a nonce survives reboots (the browser gets a
// silent stale=true re-auth) but is invalidated the moment the password or
// username changes - which is exactly the re-prompt we want then.
static String nonceSecret() {
  return md5(webAuthGetUser() + ":" + REALM + ":" + webAuthGetPass());
}

// Per-role HA1-style secret used to sign the remember-me cookie, so a cookie is
// bound to that account's credentials (and the role can't be forged into the
// other). Changing an account's password invalidates only its own sessions.
static String roleSecret(int role) {
  loadCreds();
  return role == ROLE_VIEWER ? md5(g_vuser + ":" + REALM + ":" + g_vpass)
                             : md5(g_user + ":" + REALM + ":" + g_pass);
}

static String makeNonce() {
  String ts = String((uint32_t)millis(), HEX);
  return ts + "-" + md5(ts + ":" + nonceSecret());
}

// Valid = we signed it. expired is reported separately so the caller can
// distinguish "right password, old nonce" (stale) from a bad login.
static bool nonceValid(const String &n, bool &expired) {
  int dash = n.indexOf('-');
  if (dash < 1) return false;
  String ts = n.substring(0, dash);
  if (!md5(ts + ":" + nonceSecret()).equalsIgnoreCase(n.substring(dash + 1))) return false;
  expired = (uint32_t)millis() - strtoul(ts.c_str(), NULL, 16) > NONCE_TTL_MS;
  return true;
}

// Thin String shell over the pure parser (http_parse.h, unit-tested). Digest
// param values (hex responses, nonces, our short URIs) fit comfortably.
static String dparam(const String &h, const char *key) {
  char buf[160];
  if (!httpparse::digestParam(h.c_str(), key, buf, sizeof(buf))) return "";
  return String(buf);
}

static void send401(WebServer &server, bool stale) {
  String ch = String("Digest realm=\"") + REALM + "\", qop=\"auth\", nonce=\"" +
              makeNonce() + "\"" + (stale ? ", stale=true" : "");
  server.sendHeader("WWW-Authenticate", ch);
  server.send(401, "text/plain", "auth required");
}

static const char *methodName(WebServer &server) {
  switch (server.method()) {
    case HTTP_POST:   return "POST";
    case HTTP_PUT:    return "PUT";
    case HTTP_DELETE: return "DELETE";
    case HTTP_PATCH:  return "PATCH";
    default:          return "GET";
  }
}

// Stateless session token: "<expiryEpoch>.<md5(secret:expiry)>". The secret is
// the same credential-derived value as the nonce, so changing the username or
// password invalidates every outstanding session too. If the clock is unset
// (AP mode / no NTP yet) the expiry is small and enforcement is skipped - the
// browser's Max-Age still ages the cookie out.
// "<role><expiryEpoch>.<md5(roleSecret:<role><expiry>)>". The role char ('a'/'v')
// is part of the signed material, so it can't be flipped, and the signature uses
// that role's credential secret - changing one account's password invalidates only
// its own sessions. Old 2-field cookies (pre-roles) fail the role parse below and
// trigger one harmless re-prompt.
static String makeSessionToken(int role) {
  time_t now = time(NULL);
  long base = now > CLOCK_SET_EPOCH ? (long)now : 0;
  String e = String(role == ROLE_VIEWER ? 'v' : 'a') + String(base + SESSION_SECS);
  return e + "." + md5(roleSecret(role) + ":" + e);
}

static bool sessionCookieValid(WebServer &server) {
  if (!server.hasHeader("Cookie")) return false;
  char tok[96], e[24], sig[40];
  if (!httpparse::cookieValue(server.header("Cookie").c_str(), COOKIE_NAME, tok,
                              sizeof(tok)))
    return false;
  if (!httpparse::splitToken(tok, e, sizeof(e), sig, sizeof(sig)))
    return false;
  char r = e[0];
  if (r != 'a' && r != 'v') return false; // not a role-tagged token
  int role = (r == 'v') ? ROLE_VIEWER : ROLE_ADMIN;
  if (role == ROLE_VIEWER && !webAuthViewerEnabled())
    return false; // viewer disabled since the cookie was issued
  if (!md5(roleSecret(role) + ":" + String(e)).equalsIgnoreCase(sig))
    return false; // forged, or that account's username/password changed
  long exp = atol(e + 1); // skip the role char
  time_t now = time(NULL);
  if (exp > CLOCK_SET_EPOCH && now > CLOCK_SET_EPOCH && (long)now > exp)
    return false; // expired (only enforced when both clocks are real)
  g_reqRole = role;
  return true;
}

static void issueSession(WebServer &server, int role) {
  // SameSite=Strict (not Lax): several mutating endpoints are GETs (/wifi/cfg,
  // /wifi/reboot, /wifi/add|join|del|mode, /camera/power). Lax still attaches the
  // cookie on top-level cross-site GET navigations (<a>, window.open), which is a
  // CSRF vector for those GET mutations; Strict withholds it on any cross-site
  // request. This is a same-origin LAN dashboard, so no flow depends on the cookie
  // surviving a cross-site top-level navigation.
  // THREAT MODEL: on plain-HTTP LAN this cookie (and the digest Authorization
  // header) is sniffable and replayable within its TTL/Max-Age - accepted as a
  // deliberate trade-off for a credentials-on-the-wire LAN device.
  String c = String(COOKIE_NAME) + "=" + makeSessionToken(role) +
             "; Max-Age=" + String(SESSION_SECS) + "; Path=/; HttpOnly; SameSite=Strict";
  server.sendHeader("Set-Cookie", c); // plain HTTP (LAN device): no Secure flag
}

// Register the headers the auth layer reads. Authorization is exposed by the
// WebServer by default; Cookie is not, so it must be collected explicitly.
void webAuthBegin(WebServer &server) {
  static const char *keys[] = {"Cookie", "Authorization"};
  server.collectHeaders(keys, 2);
}

bool webAuthCheck(WebServer &server) {
  if (!webAuthEnabled()) { // passwordless: open access, every request is admin
    g_reqRole = ROLE_ADMIN;
    presenceTouch((uint32_t)server.client().remoteIP());
    return true;
  }
  if (sessionCookieValid(server)) { // remembered session, no prompt
    presenceTouch((uint32_t)server.client().remoteIP());
    return true;
  }
  if (server.hasHeader("Authorization")) {
    String auth = server.header("Authorization");
    if (auth.startsWith("Digest")) {
      String nonce = dparam(auth, "nonce");
      bool expired = false;
      // Identify the account by the supplied username: admin, or (if enabled) the
      // read-only viewer. HA1 is then computed from THAT account's password.
      String reqUser = dparam(auth, "username");
      int role = ROLE_ADMIN;
      bool userOk = false;
      if (reqUser == webAuthGetUser()) { role = ROLE_ADMIN; userOk = true; }
      else if (webAuthViewerEnabled() && reqUser == webAuthGetViewerUser()) { role = ROLE_VIEWER; userOk = true; }
      if (userOk && nonceValid(nonce, expired)) {
        String ha1 = roleSecret(role); // md5(user:realm:pass) for the matched account
        String ha2 = md5(String(methodName(server)) + ":" + dparam(auth, "uri"));
        String qop = dparam(auth, "qop");
        // REPLAY THREAT MODEL: nc (nonce-count) below is fed into the RFC 2617
        // response hash because the client computes the hash over it - it is
        // load-bearing for verification. It is NOT enforced monotonically: doing
        // so needs per-nonce server state (a seen-nc set per outstanding nonce)
        // that isn't worth the RAM on this device. Consequence: a captured
        // Authorization header (or session cookie) is replayable over plain HTTP
        // for the life of its nonce/cookie TTL. Accepted deliberately - this is a
        // plain-HTTP LAN device where the credential is already on the wire.
        String expect = qop.length()
            ? md5(ha1 + ":" + nonce + ":" + dparam(auth, "nc") + ":" +
                  dparam(auth, "cnonce") + ":" + qop + ":" + ha2)
            : md5(ha1 + ":" + nonce + ":" + ha2); // RFC 2069 fallback
        if (dparam(auth, "response").equalsIgnoreCase(expect)) {
          // The digest is computed over the request URI (ha2 above), but the
          // server must also confirm that URI is THIS request's target - otherwise
          // a captured Authorization header for one endpoint (e.g. a browser's
          // routinely-polled /diag) is replayable against another (e.g. the
          // mutating /wifi/cfg). Compare on path only (pure, unit-tested in
          // test/test_http_parse: uriPathMatches strips the client uri's query).
          if (!httpparse::uriPathMatches(dparam(auth, "uri").c_str(),
                                         server.uri().c_str())) {
            send401(server, false); // uri mismatch: replay against a different path
            return false;
          }
          if (!expired) {
            g_reqRole = role;
            issueSession(server, role); // remember this browser so it stops prompting
            presenceTouch((uint32_t)server.client().remoteIP());
            return true;
          }
          send401(server, true); // good creds, old nonce: silent re-auth
          return false;
        }
      }
    }
  }
  send401(server, false);
  return false;
}

bool webAuthRequireWrite(WebServer &server) {
  if (!webAuthCheck(server)) return false; // 401 / stale handled inside
  if (webAuthIsViewer()) {
    server.send(403, "text/plain", "read-only account: not permitted");
    return false;
  }
  return true;
}
