#pragma once

#include <filesystem>
#include <regex>

namespace bulbit
{

inline std::vector<std::string> SplitString(const std::string& str, const std::regex& delim_regex)
{
    std::sregex_token_iterator first{ begin(str), end(str), delim_regex, -1 }, last;
    std::vector<std::string> list{ first, last };
    return list;
}

inline std::string ToLowercase(const std::string& s)
{
    std::string out = s;
    std::transform(s.begin(), s.end(), out.begin(), ::tolower);
    return out;
}

inline std::filesystem::path NextFileName(const std::filesystem::path& filename)
{
    namespace fs = std::filesystem;

    if (!fs::exists(filename))
    {
        return filename;
    }

    fs::path dir = filename.parent_path();
    std::string stem = filename.stem().string();
    std::string ext = filename.extension().string();

    int i = 1;
    fs::path next_filename;
    do
    {
        next_filename = dir / (stem + "_" + std::to_string(i) + ext);
        ++i;
    } while (fs::exists(next_filename));

    return next_filename;
}

} // namespace bulbit
