/*
 * Amazon FreeRTOS V201906.00 Major
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

#ifndef __AWS_CODESIGN_KEYS__H__
#define __AWS_CODESIGN_KEYS__H__

/*
 * PEM-encoded code signer certificate
 *
 * Must include the PEM header and footer:
 * "-----BEGIN CERTIFICATE-----\n"
 * "...base64 data...\n"
 * "-----END CERTIFICATE-----\n";
 */
static const char signingcredentialSIGNING_CERTIFICATE_PEM[] = "-----BEGIN CERTIFICATE-----\n" \
                        "MIIBWjCCAQCgAwIBAgIUMbotBhWpNJfv+UY290HWvTYthaMwCgYIKoZIzj0EAwIw\n"   \
                        "GjEYMBYGA1UEAwwPbml4bG9uZ0AxMjYuY29tMB4XDTE5MDgyMDA5MjE1NVoXDTIw\n"   \
                        "MDgxOTA5MjE1NVowGjEYMBYGA1UEAwwPbml4bG9uZ0AxMjYuY29tMFkwEwYHKoZI\n"   \
                        "zj0CAQYIKoZIzj0DAQcDQgAENgBvf7IAI/k6pR+yd1ck1GwXkXJTuQsDiaxKdFPM\n"   \
                        "N2gzKFOoVWYEz9GKM+Ml4UbZ2gHVCDtwkl6KxC8uIreyLaMkMCIwCwYDVR0PBAQD\n"   \
                        "AgeAMBMGA1UdJQQMMAoGCCsGAQUFBwMDMAoGCCqGSM49BAMCA0gAMEUCIQC5oT2i\n"   \
                        "4jHGZJiA02SJHgvKT42K0u+wZhe3aGWHYtH7ugIgaaNCt4CWyw2mzCfjyOa0sBxb\n"   \
                        "4oBqnYF1urXMfJsOIE4=\n"                                               \
                        "-----END CERTIFICATE-----";

#endif
