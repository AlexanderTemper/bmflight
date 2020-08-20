#include "config/feature.h"

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
    return featureConfig()->enabledFeatures & mask;
}

void featureEnable(const uint32_t mask)
{
    featureSet(mask, &featureConfigMutable()->enabledFeatures);
}

void featureDisable(const uint32_t mask)
{
    featureClear(mask, &featureConfigMutable()->enabledFeatures);
}

void featureDisableAll(void)
{
    featureConfigMutable()->enabledFeatures = 0;
}

uint32_t featureMask(void)
{
    return featureConfig()->enabledFeatures;
}
