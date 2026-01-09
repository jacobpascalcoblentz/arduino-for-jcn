/**
 * @file test_yaml_config.cpp
 * @brief Google Test unit tests for YAML configuration parser
 */

#include <gtest/gtest.h>
#include <cstdio>
#include <cstring>
#include "../src/yaml_config.h"

// Helper to create test config files
class YamlConfigTestFixture : public ::testing::Test {
protected:
    const char* testConfigFile = "test_config.yaml";

    void SetUp() override {
        // Create a test config file
        FILE* f = fopen(testConfigFile, "w");
        if (f) {
            fprintf(f, "# Test configuration\n");
            fprintf(f, "setpoints:\n");
            fprintf(f, "  ph: 6.2\n");
            fprintf(f, "  tds: 750\n");
            fprintf(f, "  temperature: 23.5\n");
            fprintf(f, "  water_level: 75\n");
            fprintf(f, "\n");
            fprintf(f, "safety:\n");
            fprintf(f, "  ph_min: 5.0\n");
            fprintf(f, "  ph_max: 7.5\n");
            fprintf(f, "\n");
            fprintf(f, "system:\n");
            fprintf(f, "  volume_liters: 50\n");
            fprintf(f, "\n");
            fprintf(f, "logging:\n");
            fprintf(f, "  enabled: true\n");
            fprintf(f, "  filename: \"test.csv\"\n");
            fclose(f);
        }
    }

    void TearDown() override {
        remove(testConfigFile);
    }
};

// ============================================================================
// HYDRO CONFIG DEFAULTS TESTS
// ============================================================================

TEST(HydroConfigTest, DefaultValues) {
    HydroConfig config;
    config.setDefaults();

    EXPECT_FLOAT_EQ(config.phSetpoint, 6.5f);
    EXPECT_FLOAT_EQ(config.tdsSetpoint, 800.0f);
    EXPECT_FLOAT_EQ(config.temperatureSetpoint, 24.0f);
    EXPECT_FLOAT_EQ(config.waterLevelSetpoint, 80.0f);
}

TEST(HydroConfigTest, SafetyDefaults) {
    HydroConfig config;
    config.setDefaults();

    EXPECT_FLOAT_EQ(config.phMin, 5.5f);
    EXPECT_FLOAT_EQ(config.phMax, 7.5f);
    EXPECT_GT(config.tdsMin, 0.0f);
    EXPECT_GT(config.tdsMax, config.tdsMin);
}

TEST(HydroConfigTest, LogFilenameDefault) {
    HydroConfig config;
    config.setDefaults();

    EXPECT_STREQ(config.logFilename, "hydro.csv");
    EXPECT_TRUE(config.loggingEnabled);
}

TEST(HydroConfigTest, LoadedFlagDefault) {
    HydroConfig config;
    config.setDefaults();

    EXPECT_FALSE(config.loaded);
    EXPECT_EQ(config.loadError[0], '\0');
}

// ============================================================================
// YAML PARSER TESTS
// ============================================================================

TEST_F(YamlConfigTestFixture, LoadConfigFile) {
    YamlConfigParser parser;
    HydroConfig config;

    bool success = parser.load(testConfigFile, config);

    EXPECT_TRUE(success);
    EXPECT_TRUE(config.loaded);
}

TEST_F(YamlConfigTestFixture, ParseSetpoints) {
    YamlConfigParser parser;
    HydroConfig config;

    parser.load(testConfigFile, config);

    EXPECT_FLOAT_EQ(config.phSetpoint, 6.2f);
    EXPECT_FLOAT_EQ(config.tdsSetpoint, 750.0f);
    EXPECT_FLOAT_EQ(config.temperatureSetpoint, 23.5f);
    EXPECT_FLOAT_EQ(config.waterLevelSetpoint, 75.0f);
}

TEST_F(YamlConfigTestFixture, ParseSafetyLimits) {
    YamlConfigParser parser;
    HydroConfig config;

    parser.load(testConfigFile, config);

    EXPECT_FLOAT_EQ(config.phMin, 5.0f);
    EXPECT_FLOAT_EQ(config.phMax, 7.5f);
}

TEST_F(YamlConfigTestFixture, ParseSystemParams) {
    YamlConfigParser parser;
    HydroConfig config;

    parser.load(testConfigFile, config);

    EXPECT_FLOAT_EQ(config.systemVolumeLiters, 50.0f);
}

TEST_F(YamlConfigTestFixture, ParseBooleanTrue) {
    YamlConfigParser parser;
    HydroConfig config;

    parser.load(testConfigFile, config);

    EXPECT_TRUE(config.loggingEnabled);
}

TEST_F(YamlConfigTestFixture, ParseQuotedString) {
    YamlConfigParser parser;
    HydroConfig config;

    parser.load(testConfigFile, config);

    EXPECT_STREQ(config.logFilename, "test.csv");
}

TEST_F(YamlConfigTestFixture, MissingFileUsesDefaults) {
    YamlConfigParser parser;
    HydroConfig config;

    bool success = parser.load("nonexistent.yaml", config);

    EXPECT_FALSE(success);
    // Should still have defaults
    EXPECT_FLOAT_EQ(config.phSetpoint, 6.5f);  // Default value
}

TEST_F(YamlConfigTestFixture, HasKey) {
    YamlConfigParser parser;
    HydroConfig config;

    parser.load(testConfigFile, config);

    EXPECT_TRUE(parser.hasKey("setpoints.ph"));
    EXPECT_TRUE(parser.hasKey("safety.ph_min"));
    EXPECT_FALSE(parser.hasKey("nonexistent.key"));
}

TEST_F(YamlConfigTestFixture, GetFloatWithDefault) {
    YamlConfigParser parser;
    HydroConfig config;

    parser.load(testConfigFile, config);

    float existingVal = parser.getFloat("setpoints.ph", 99.0f);
    float missingVal = parser.getFloat("nonexistent.key", 99.0f);

    EXPECT_FLOAT_EQ(existingVal, 6.2f);
    EXPECT_FLOAT_EQ(missingVal, 99.0f);
}

TEST_F(YamlConfigTestFixture, GetIntWithDefault) {
    YamlConfigParser parser;
    HydroConfig config;

    parser.load(testConfigFile, config);

    int existingVal = parser.getInt("setpoints.tds", 999);
    int missingVal = parser.getInt("nonexistent.key", 999);

    EXPECT_EQ(existingVal, 750);
    EXPECT_EQ(missingVal, 999);
}

// ============================================================================
// CONFIG VALIDATION TESTS
// ============================================================================

TEST(ConfigValidation, ValidConfig) {
    HydroConfig config;
    config.setDefaults();

    char error[64];
    bool valid = ConfigLoader::validateConfig(config, error);

    EXPECT_TRUE(valid);
    EXPECT_EQ(error[0], '\0');
}

TEST(ConfigValidation, InvalidPhSetpoint) {
    HydroConfig config;
    config.setDefaults();
    config.phSetpoint = 15.0f;  // Invalid (> 14)

    char error[64];
    bool valid = ConfigLoader::validateConfig(config, error);

    EXPECT_FALSE(valid);
    EXPECT_STRNE(error, "");
}

TEST(ConfigValidation, InvalidPhRange) {
    HydroConfig config;
    config.setDefaults();
    config.phMin = 7.0f;
    config.phMax = 5.0f;  // Min > Max

    char error[64];
    bool valid = ConfigLoader::validateConfig(config, error);

    EXPECT_FALSE(valid);
}

TEST(ConfigValidation, InvalidTdsSetpoint) {
    HydroConfig config;
    config.setDefaults();
    config.tdsSetpoint = 5000.0f;  // Too high

    char error[64];
    bool valid = ConfigLoader::validateConfig(config, error);

    EXPECT_FALSE(valid);
}

TEST(ConfigValidation, InvalidTemperature) {
    HydroConfig config;
    config.setDefaults();
    config.temperatureSetpoint = 50.0f;  // Too high

    char error[64];
    bool valid = ConfigLoader::validateConfig(config, error);

    EXPECT_FALSE(valid);
}

TEST(ConfigValidation, ZeroVolume) {
    HydroConfig config;
    config.setDefaults();
    config.systemVolumeLiters = 0.0f;

    char error[64];
    bool valid = ConfigLoader::validateConfig(config, error);

    EXPECT_FALSE(valid);
}

// ============================================================================
// YAML PARSER EDGE CASES
// ============================================================================

TEST(YamlParserEdgeCases, CommentHandling) {
    // Create a file with various comment styles
    const char* filename = "comment_test.yaml";
    FILE* f = fopen(filename, "w");
    fprintf(f, "# Full line comment\n");
    fprintf(f, "setpoints:\n");
    fprintf(f, "  ph: 6.0  # Inline comment\n");
    fprintf(f, "  # Indented comment\n");
    fprintf(f, "  tds: 800\n");
    fclose(f);

    YamlConfigParser parser;
    HydroConfig config;
    parser.load(filename, config);

    EXPECT_FLOAT_EQ(config.phSetpoint, 6.0f);
    EXPECT_FLOAT_EQ(config.tdsSetpoint, 800.0f);

    remove(filename);
}

TEST(YamlParserEdgeCases, EmptyLines) {
    const char* filename = "empty_test.yaml";
    FILE* f = fopen(filename, "w");
    fprintf(f, "\n\n");
    fprintf(f, "setpoints:\n");
    fprintf(f, "\n");
    fprintf(f, "  ph: 6.0\n");
    fprintf(f, "\n\n");
    fclose(f);

    YamlConfigParser parser;
    HydroConfig config;
    parser.load(filename, config);

    EXPECT_FLOAT_EQ(config.phSetpoint, 6.0f);

    remove(filename);
}

TEST(YamlParserEdgeCases, WhitespaceHandling) {
    const char* filename = "whitespace_test.yaml";
    FILE* f = fopen(filename, "w");
    fprintf(f, "setpoints:\n");
    fprintf(f, "  ph:    6.5   \n");  // Extra spaces
    fprintf(f, "  tds:800\n");        // No space after colon
    fclose(f);

    YamlConfigParser parser;
    HydroConfig config;
    parser.load(filename, config);

    EXPECT_FLOAT_EQ(config.phSetpoint, 6.5f);
    EXPECT_FLOAT_EQ(config.tdsSetpoint, 800.0f);

    remove(filename);
}

TEST(YamlParserEdgeCases, BooleanVariants) {
    const char* filename = "bool_test.yaml";
    FILE* f = fopen(filename, "w");
    fprintf(f, "test:\n");
    fprintf(f, "  val1: true\n");
    fprintf(f, "  val2: True\n");
    fprintf(f, "  val3: TRUE\n");
    fprintf(f, "  val4: yes\n");
    fprintf(f, "  val5: 1\n");
    fprintf(f, "  val6: false\n");
    fclose(f);

    YamlConfigParser parser;
    HydroConfig config;
    parser.load(filename, config);

    EXPECT_TRUE(parser.getBool("test.val1", false));
    EXPECT_TRUE(parser.getBool("test.val2", false));
    EXPECT_TRUE(parser.getBool("test.val3", false));
    EXPECT_TRUE(parser.getBool("test.val4", false));
    EXPECT_TRUE(parser.getBool("test.val5", false));
    EXPECT_FALSE(parser.getBool("test.val6", true));

    remove(filename);
}

TEST(YamlParserEdgeCases, NegativeNumbers) {
    const char* filename = "negative_test.yaml";
    FILE* f = fopen(filename, "w");
    fprintf(f, "test:\n");
    fprintf(f, "  negative: -5.5\n");
    fclose(f);

    YamlConfigParser parser;
    HydroConfig config;
    parser.load(filename, config);

    float val = parser.getFloat("test.negative", 0.0f);
    EXPECT_FLOAT_EQ(val, -5.5f);

    remove(filename);
}
