// Copyright (c) Astradyn Systems LLP.
// SPDX-License-Identifier: Apache-2.0

#ifndef ASTRATAG_EMBEDDED_DATA_HPP
#define ASTRATAG_EMBEDDED_DATA_HPP

#include <string>

namespace astratag {
namespace embedded {

/**
 * Embedded marker dictionary JSON data.
 * Contains all marker definitions with signatures and world points.
 * Automatically generated from data/new_dictionary.json at build time.
 */
extern const char* DICTIONARY_JSON;

/**
 * Embedded keypoints data.
 * Contains 48 triangular sampling regions for signature extraction.
 * Automatically generated from data/keypoints.txt at build time.
 */
extern const char* KEYPOINTS_TXT;

} // namespace embedded
} // namespace astratag

#endif // ASTRATAG_EMBEDDED_DATA_HPP
