#include "StringUtils.h"
#include "ctype.h"
#include "cstring"
#include "sstream"
#include "string"
#include <iostream>

namespace StringUtils{

std::string Slice(const std::string &str, ssize_t start, ssize_t end) noexcept {
    if (end == 0) {
        end = str.length();
    }
    if (start < 0) start += str.length();
    if (end < 0) end += str.length();
    return str.substr(start, end - start);
}

std::string Capitalize(const std::string &str) noexcept{
    if(str.empty()) {
        return str;
    }
    std::string result = str;
    result[0] = toupper(result[0]); 
    for (size_t i = 1; i < result.size(); i++) {
        result[i] = tolower(result[i]);
    }
    return result;
}

std::string Upper(const std::string &str) noexcept{
    if(str.empty()) {
        return str;
    }
    std::string result = str;
    for (size_t i = 0; i < result.size(); i++) {
        result[i] = toupper(result[i]);
    }
    return result;
}

std::string Lower(const std::string &str) noexcept{
    if(str.empty()) {
        return str;
    }
    std::string result = str;
    for (size_t i = 0; i < result.size(); i++) {
        result[i] = tolower(result[i]);
    }
    return result;
}

std::string LStrip(const std::string &str) noexcept{
    size_t start = str.find_first_not_of(" \t\n\r");
    return (start == std::string::npos) ? "" : str.substr(start);
}

std::string RStrip(const std::string &str) noexcept{
    size_t end = str.find_last_not_of(" \t\n\r");
    return (end == std::string::npos) ? "" : str.substr(0, end + 1);
}

std::string Strip(const std::string &str) noexcept{
    return LStrip(RStrip(str));
}

std::string Center(const std::string &str, int width, char fill) noexcept{
    int len = str.length();
    int num_fill = width - len;
    if (num_fill > 0){
        int left_fill = num_fill / 2;
        int right_fill = num_fill - left_fill;
        return std::string(left_fill, fill) + str + std::string(right_fill, fill);
    } else {
        return str;
    }
}

std::string LJust(const std::string &str, int width, char fill) noexcept {
    if (str.length() >= width) {
        return str;
    }
    return str + std::string(width - str.length(), fill);
}

std::string RJust(const std::string &str, int width, char fill) noexcept {
    if (str.length() >= width) {
        return str;
    }
    return std::string(width - str.length(), fill) + str;
}

std::string Replace(const std::string &str, const std::string &old, const std::string &rep) noexcept{
    std::string result = str;
    size_t pos = 0;
    while ((pos = result.find(old, pos)) != std::string::npos) {
        result.replace(pos, old.length(), rep);
        pos += rep.length();
    }
    return result;
}

std::vector< std::string > Split(const std::string &str, const std::string &splt) noexcept{
    std::vector<std::string> result;
    size_t start = 0, end;

    if (splt.empty()) {
        while ((end = str.find_first_of(" \t\n\r", start)) != std::string::npos) {
            if (end != start) result.push_back(str.substr(start, end - start));
            start = str.find_first_not_of(" \t\n\r", end);
        }
    } else {
        while ((end = str.find(splt, start)) != std::string::npos) {
            result.push_back(str.substr(start, end - start));
            start = end + splt.length();
        }
    }

    if (start < str.size()) result.push_back(str.substr(start));
    return result;
}

std::string Join(const std::string &str, const std::vector< std::string > &vect) noexcept{
    if (vect.empty()) {
        return "";
    }
    std::string result = vect[0];
    for (size_t i = 1; i < vect.size(); ++i) {
        result += str + vect[i];
    }
    return result;
}

std::string ExpandTabs(const std::string &str, int tabsize) noexcept
    {
        std::string outstring;
        // If tabsize is less than 1, then just remove the tabs
        if (tabsize < 1)
        {
            for (char c : str)
            {
                if (!(c == '\t'))
                {
                    outstring += c;
                }
            }
            return outstring;
        }
        // Otherwise, replace the tabs with spaces
        for (char c : str)
        {
            if (!(c == '\t'))
            {
                outstring += c;
            }
            else
            {
                int curr_len = outstring.length();
                // Calculate the number of spaces needed to reach the next tabstop for alignment
                int num_spaces = tabsize - (outstring.length() % tabsize);
                outstring += std::string(num_spaces, ' ');
            }
        }
        return outstring;
    }

int EditDistance(const std::string &left, const std::string &right, bool ignorecase) noexcept{
    std::string l = ignorecase ? Lower(left) : left;
    std::string r = ignorecase ? Lower(right) : right;
    
    size_t len1 = l.size(), len2 = r.size();
    std::vector<std::vector<int>> dist(len1 + 1, std::vector<int>(len2 + 1));

    for (size_t i = 0; i <= len1; ++i) dist[i][0] = i;
    for (size_t j = 0; j <= len2; ++j) dist[0][j] = j;

    for (size_t i = 1; i <= len1; ++i) {
        for (size_t j = 1; j <= len2; ++j) {
            int cost = (l[i - 1] == r[j - 1]) ? 0 : 1;
            dist[i][j] = std::min(dist[i][j - 1] + 1, std::min(dist[i - 1][j] + 1, dist[i - 1][j - 1] + cost));
        }
    }
    return dist[len1][len2];
}

};