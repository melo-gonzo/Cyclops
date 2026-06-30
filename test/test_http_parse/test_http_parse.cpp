// Host-native unit tests for the pure HTTP auth-header parsing. pio test -e native

#include <unity.h>
#include <string.h>
#include "http_parse.h"

using namespace httpparse;

static char out[160];

// A realistic Digest Authorization header.
static const char *DIGEST =
    "Digest username=\"admin\", realm=\"Cyclops\", "
    "nonce=\"1a2b-deadbeef\", uri=\"/audio/status\", qop=auth, nc=00000001, "
    "cnonce=\"f00d\", response=\"abcdef0123456789abcdef0123456789\"";

// --- whole-key matching: the historic bug ------------------------------------
void test_nonce_not_matched_inside_cnonce(void) {
  TEST_ASSERT_TRUE(digestParam(DIGEST, "nonce", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("1a2b-deadbeef", out); // NOT "f00d" from cnonce
}
void test_cnonce_extracted(void) {
  TEST_ASSERT_TRUE(digestParam(DIGEST, "cnonce", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("f00d", out);
}

// --- quoted vs bare token values ---------------------------------------------
void test_quoted_value(void) {
  TEST_ASSERT_TRUE(digestParam(DIGEST, "username", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("admin", out);
}
void test_bare_token_value_ends_at_comma(void) {
  TEST_ASSERT_TRUE(digestParam(DIGEST, "qop", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("auth", out);
}
void test_bare_token_value_nc(void) {
  TEST_ASSERT_TRUE(digestParam(DIGEST, "nc", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("00000001", out);
}
void test_response_hex(void) {
  TEST_ASSERT_TRUE(digestParam(DIGEST, "response", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("abcdef0123456789abcdef0123456789", out);
}

// --- missing / malformed ------------------------------------------------------
void test_missing_key_returns_false(void) {
  TEST_ASSERT_FALSE(digestParam(DIGEST, "opaque", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("", out);
}
void test_unterminated_quote_is_absent(void) {
  TEST_ASSERT_FALSE(digestParam("x=\"oops", "x", out, sizeof(out)));
}
void test_key_as_substring_prefix_not_matched(void) {
  // "user" must not match "username" (no '=' right after).
  TEST_ASSERT_FALSE(digestParam(DIGEST, "user", out, sizeof(out)));
}

// --- cookies -----------------------------------------------------------------
void test_cookie_value_extracted(void) {
  TEST_ASSERT_TRUE(cookieValue("foo=1; cyc_auth=tok.sig; bar=2", "cyc_auth",
                               out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("tok.sig", out);
}
void test_cookie_value_at_end(void) {
  TEST_ASSERT_TRUE(cookieValue("a=1; cyc_auth=zzz", "cyc_auth", out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("zzz", out);
}
void test_cookie_missing(void) {
  TEST_ASSERT_FALSE(cookieValue("a=1; b=2", "cyc_auth", out, sizeof(out)));
}
void test_cookie_false_prefix_not_matched(void) {
  // "xcyc_auth" must NOT satisfy a lookup for "cyc_auth": the real one wins.
  TEST_ASSERT_TRUE(cookieValue("xcyc_auth=bad; cyc_auth=good", "cyc_auth",
                               out, sizeof(out)));
  TEST_ASSERT_EQUAL_STRING("good", out);
}

// --- token split -------------------------------------------------------------
void test_split_token(void) {
  char e[24], sig[40];
  TEST_ASSERT_TRUE(splitToken("1700000123.deadbeef", e, sizeof(e), sig, sizeof(sig)));
  TEST_ASSERT_EQUAL_STRING("1700000123", e);
  TEST_ASSERT_EQUAL_STRING("deadbeef", sig);
}
void test_split_token_rejects_no_dot(void) {
  char e[24], sig[40];
  TEST_ASSERT_FALSE(splitToken("nodot", e, sizeof(e), sig, sizeof(sig)));
}
void test_split_token_rejects_leading_dot(void) {
  char e[24], sig[40];
  TEST_ASSERT_FALSE(splitToken(".sigonly", e, sizeof(e), sig, sizeof(sig)));
}

// ---- uriPathMatches: digest uri vs the request path (replay-across-paths guard) ----
void test_uri_exact_match(void) {
  TEST_ASSERT_TRUE(uriPathMatches("/diag", "/diag"));
}
void test_uri_query_is_stripped_before_compare(void) {
  // the client signs the full request-target incl. query; server path has none
  TEST_ASSERT_TRUE(uriPathMatches("/camera/config?var=framesize&val=8", "/camera/config"));
}
void test_uri_different_path_rejected(void) {
  // a /diag header replayed against /wifi/cfg must NOT validate
  TEST_ASSERT_FALSE(uriPathMatches("/diag", "/wifi/cfg"));
}
void test_uri_prefix_is_not_a_match(void) {
  TEST_ASSERT_FALSE(uriPathMatches("/wifi", "/wifi/cfg"));      // shorter
  TEST_ASSERT_FALSE(uriPathMatches("/wifi/cfg", "/wifi"));      // longer
}
void test_uri_query_on_wrong_path_rejected(void) {
  TEST_ASSERT_FALSE(uriPathMatches("/diag?x=1", "/wifi/cfg"));
}
void test_uri_null_is_false(void) {
  TEST_ASSERT_FALSE(uriPathMatches(nullptr, "/x"));
  TEST_ASSERT_FALSE(uriPathMatches("/x", nullptr));
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_nonce_not_matched_inside_cnonce);
  RUN_TEST(test_cnonce_extracted);
  RUN_TEST(test_quoted_value);
  RUN_TEST(test_bare_token_value_ends_at_comma);
  RUN_TEST(test_bare_token_value_nc);
  RUN_TEST(test_response_hex);
  RUN_TEST(test_missing_key_returns_false);
  RUN_TEST(test_unterminated_quote_is_absent);
  RUN_TEST(test_key_as_substring_prefix_not_matched);
  RUN_TEST(test_cookie_value_extracted);
  RUN_TEST(test_cookie_value_at_end);
  RUN_TEST(test_cookie_missing);
  RUN_TEST(test_cookie_false_prefix_not_matched);
  RUN_TEST(test_split_token);
  RUN_TEST(test_split_token_rejects_no_dot);
  RUN_TEST(test_split_token_rejects_leading_dot);
  RUN_TEST(test_uri_exact_match);
  RUN_TEST(test_uri_query_is_stripped_before_compare);
  RUN_TEST(test_uri_different_path_rejected);
  RUN_TEST(test_uri_prefix_is_not_a_match);
  RUN_TEST(test_uri_query_on_wrong_path_rejected);
  RUN_TEST(test_uri_null_is_false);
  return UNITY_END();
}
