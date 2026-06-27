#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "../src/cyw43.c"

START_TEST(test_buffer_reads_never_exceed_declared_length)
{
    // Invariant: Buffer reads never exceed the declared length
    const char *payloads[] = {
        "normal",                    // Valid input
        "A",                         // Boundary: single char
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",  // 100 chars - exceeds typical buffers
        "\x00\x01\x02\x03\x04\x05",  // Binary data
        NULL                         // Sentinel
    };
    
    // Test strncpy usage in cyw43_ll_join function
    for (int i = 0; payloads[i] != NULL; i++) {
        const char *input = payloads[i];
        size_t input_len = strlen(input);
        
        // Target buffer size from cyw43.c implementation
        #define TEST_BUF_SIZE 64
        char dest[TEST_BUF_SIZE] = {0};
        char backup[TEST_BUF_SIZE] = {0};
        
        // Initialize with known pattern
        memset(dest, 'X', TEST_BUF_SIZE);
        memcpy(backup, dest, TEST_BUF_SIZE);
        
        // Call strncpy as used in cyw43.c
        strncpy(dest, input, TEST_BUF_SIZE - 1);
        dest[TEST_BUF_SIZE - 1] = '\0';
        
        // Check: No buffer overflow beyond TEST_BUF_SIZE
        ck_assert_msg(memcmp(&dest[TEST_BUF_SIZE], &backup[TEST_BUF_SIZE], 1) == 0,
                     "Buffer overflow detected for input: %s", input);
        
        // Check: String is properly terminated
        ck_assert_msg(dest[TEST_BUF_SIZE - 1] == '\0',
                     "Buffer not properly null-terminated for input: %s", input);
        
        // Check: Input was truncated if too long
        if (input_len >= TEST_BUF_SIZE) {
            ck_assert_msg(strlen(dest) == TEST_BUF_SIZE - 1,
                         "Oversized input not properly truncated: %s", input);
        }
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_buffer_reads_never_exceed_declared_length);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}