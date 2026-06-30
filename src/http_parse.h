// http_parse.h
//
// Pure, hardware-free HTTP header parsing for the auth layer: extracting a
// Digest "key=value" parameter and a named cookie value. No Arduino includes
// (works on plain C strings), so it is unit-tested on the host (test/test_http_parse).
// web_auth.cpp is the shell that adapts Arduino String <-> these.
//
// These had subtle, security-relevant bugs by hand (matching "nonce" inside
// "cnonce"); pinning them with tests is the point.

#pragma once

#include <stddef.h>
#include <string.h>

namespace httpparse {

// Extract a Digest "key=value" from a WWW-Authenticate/Authorization header.
// Matches WHOLE keys only (the char before key must be start, space, or comma,
// and '=' must immediately follow) so "nonce" never matches inside "cnonce".
// Values may be quoted ("...") or bare tokens (ended by ',' or ' '). A quoted
// value with no closing quote is treated as absent. Writes a NUL-terminated
// result into out and returns true on a found, well-formed value.
inline bool digestParam(const char *header, const char *key, char *out,
                        size_t outsz) {
  if (!header || !key || !out || outsz == 0) return false;
  out[0] = '\0';
  size_t klen = strlen(key);
  if (klen == 0) return false;

  const char *p = header;
  const char *found = nullptr;
  while ((p = strstr(p, key)) != nullptr) {
    char prev = (p == header) ? ' ' : p[-1];
    if (p[klen] == '=' && (prev == ' ' || prev == ',')) { found = p; break; }
    p++; // overlap-safe advance
  }
  if (!found) return false;

  const char *v = found + klen + 1; // past "key="
  size_t n = 0;
  if (*v == '"') {
    v++;
    const char *end = strchr(v, '"');
    if (!end) return false; // unterminated quote -> treat as absent
    while (v < end && n < outsz - 1) out[n++] = *v++;
  } else {
    while (*v && *v != ',' && *v != ' ' && n < outsz - 1) out[n++] = *v++;
  }
  out[n] = '\0';
  return true;
}

// Extract a cookie value ("name=value" up to ';' or end) from a Cookie header.
// Matches WHOLE cookie names only: '=' must immediately follow the name, AND the
// char before it must be start-of-string, ';', or ' ' (the cookie-pair
// delimiters) so "cyc_auth" is not matched inside "xcyc_auth". Writes
// NUL-terminated; returns true if found.
inline bool cookieValue(const char *cookies, const char *name, char *out,
                        size_t outsz) {
  if (!cookies || !name || !out || outsz == 0) return false;
  out[0] = '\0';
  size_t nlen = strlen(name);
  if (nlen == 0) return false;

  const char *p = cookies;
  const char *found = nullptr;
  while ((p = strstr(p, name)) != nullptr) {
    char prev = (p == cookies) ? ';' : p[-1];
    if (p[nlen] == '=' && (prev == ';' || prev == ' ')) { found = p; break; }
    p++; // overlap-safe advance
  }
  if (!found) return false;

  const char *v = found + nlen + 1; // past "name="
  size_t n = 0;
  while (*v && *v != ';' && n < outsz - 1) out[n++] = *v++;
  out[n] = '\0';
  return true;
}

// Does the URI from a Digest "uri=" parameter address the given request path?
// The client's digest uri is the full request-target (it may carry a "?query");
// the server's path (Arduino WebServer::uri()) has the query stripped. Compare on
// the path only - so a captured Authorization header for one endpoint can't be
// replayed against a different path (the digest response itself can't be forged,
// but without this check the server accepts whatever uri the client signed).
inline bool uriPathMatches(const char *digestUri, const char *reqPath) {
  if (!digestUri || !reqPath) return false;
  const char *q = strchr(digestUri, '?');
  size_t plen = q ? (size_t)(q - digestUri) : strlen(digestUri);
  return plen == strlen(reqPath) && memcmp(digestUri, reqPath, plen) == 0;
}

// Split a session token "<expiry>.<sig>" at the first '.'. Both parts written
// NUL-terminated. Returns true only when the dot is at index >= 1 (non-empty
// expiry) - matching the issuer's format.
inline bool splitToken(const char *tok, char *expiry, size_t exsz, char *sig,
                       size_t sigsz) {
  if (!tok || !expiry || !sig || exsz == 0 || sigsz == 0) return false;
  const char *dot = strchr(tok, '.');
  if (!dot || dot == tok) return false; // need dot at index >= 1
  size_t elen = (size_t)(dot - tok);
  if (elen >= exsz) return false;
  memcpy(expiry, tok, elen);
  expiry[elen] = '\0';
  size_t slen = strlen(dot + 1);
  if (slen >= sigsz) return false;
  memcpy(sig, dot + 1, slen + 1);
  return true;
}

} // namespace httpparse
