inline bool SampleActive(int count)
{
    bool minFrames = count > 50;
    bool modFrames = count % 20 == 0;

    return (minFrames && modFrames);
}

inline bool FusionActive(int count)
{
    bool frameWindow1 = count > 750 && count < 1200;
    bool frameWindow2 = count > 1200 && count < 1400;

    return (!frameWindow1 && !frameWindow2 );
}

inline bool FusionActiveOld(int count)
{
    bool frameWindow1 = count > 750 && count < 1200;
    bool frameWindow2 = count > 1300 && count < 1600;
    bool minFrames = count > 150;
    bool modFrames = count % 20 == 0;

    return (minFrames && modFrames && !frameWindow1 && !frameWindow2);
}
