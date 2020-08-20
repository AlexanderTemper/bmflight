#include "config/feature.h"

featureConfig_t featureConfig;

void featureSet(const uint32_t mask, uint32_t *features)
{
    *features |= mask;
}

void featureClear(const uint32_t mask, uint32_t *features)
{
    *features &= ~(mask);
}

bool featureIsEnabled(const uint32_t mask)
{
    return featureConfig.enabledFeatures & mask;
}

void featureEnable(const uint32_t mask)
{
    featureSet(mask, &featureConfig.enabledFeatures);
}

void featureDisable(const uint32_t mask)
{
    featureClear(mask, &featureConfig.enabledFeatures);
}

void featureDisableAll(void)
{
    featureConfig.enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return  featureConfig.enabledFeatures;
}
