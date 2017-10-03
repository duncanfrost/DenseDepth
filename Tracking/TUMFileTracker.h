#pragma once

#include <vector>
#include <string>
#include <sophus/se3.hpp>
#include "FileTracker.h"

class TUMFileTracker : public FileTracker
{
public:
    TUMFileTracker(const std::string &directory, const std::string &listName);

private:
    void Init(const std::string &directory, const std::string &listName);
};
