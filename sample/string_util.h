#pragma once

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

} // namespace bulbit
