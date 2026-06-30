// web_auth.h
//
// HTTP Digest authentication (RFC 2617) with stateless nonces.
//
// The stock WebServer::authenticate() keeps only the single most recent
// nonce it issued, so any two clients (or parallel fetches from one
// dashboard tab) invalidate each other and browsers re-prompt forever.
// Here every nonce is self-validating - "<millis hex>-<md5(ts:secret)>" -
// so unlimited clients can authenticate concurrently with no shared state.
// Expired nonces answer with stale=true, which browsers honor by silently
// re-authenticating instead of prompting.

#pragma once

#include <WebServer.h>

// Register the headers the auth layer needs (Cookie for the remember-me
// session). Call once before server.begin().
void webAuthBegin(WebServer &server);

// Returns true if the request carries valid Digest credentials OR a valid
// remember-me session cookie. Otherwise sends the 401 challenge itself and
// returns false. A successful digest login also sets the session cookie so the
// browser is not prompted again until it expires (or the password changes).
// PASSWORDLESS: if no admin password is set (webAuthEnabled() == false), this
// returns true immediately with no prompt and the request is treated as admin.
bool webAuthCheck(WebServer &server);

// True when an admin password is set. False = passwordless mode (the factory
// default): webAuthCheck() admits everyone as admin. Set a password via
// webAuthSetCreds() to turn login on; webAuthDisable() turns it back off.
bool webAuthEnabled();

// Current web-login credentials. NVS-backed; default to DEVICE_DEFAULT_USER and
// an EMPTY password (passwordless) until changed via webAuthSetCreds().
String webAuthGetUser();
String webAuthGetPass();

// Persist new credentials to NVS (namespace "auth"). An empty argument leaves
// that field unchanged (so the username can be changed without retyping the
// password). Setting a non-empty password turns auth on. Changing either value
// invalidates outstanding nonces, so connected browsers are prompted to sign in
// again. To go back to passwordless, use webAuthDisable() (not an empty pass).
void webAuthSetCreds(const String &user, const String &pass);

// Clear the admin password -> passwordless mode (keeps the username).
void webAuthDisable();

// ---- Read-only "viewer" account ----
// A second login that can view pages, watch the stream, and listen, but cannot
// change anything. Disabled until an admin sets a username (empty user = off).
// Stored in the same "auth" NVS namespace (keys "vu"/"vp").
void   webAuthSetViewerCreds(const String &user, const String &pass);
String webAuthGetViewerUser();
bool   webAuthViewerEnabled();

// True if the request that just passed webAuthCheck() authenticated as the
// read-only viewer (vs the admin). Valid only immediately after webAuthCheck()
// on the single-threaded server task.
bool webAuthIsViewer();

// Like webAuthCheck(), but additionally rejects the read-only viewer: on a
// viewer it sends 403 and returns false. Use to gate every mutating endpoint
// (config/trigger/delete/cfg/reset/update/...). Admin and a fresh 401 both pass
// through webAuthCheck()'s own handling.
bool webAuthRequireWrite(WebServer &server);
