#pragma once
#include <string>
#include <vector>

class PhoneSource
{
public:
    enum ImageType {LEFT, RIGHT, DEPTH};

private:
    void StereoPathsFromListFile(std::vector<std::string> &imagePaths,
                                 std::vector<long long> &timeStamps,
                                 std::string listPath,
                                 ImageType imageType);

};
