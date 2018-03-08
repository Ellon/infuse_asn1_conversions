#include "conversions/asn1_conversions.hpp" 

#include <cstring>   // for memcpy
#include <stdexcept> // for std::runtime_error

void toASN1SCC(const std::string& str, T_String& t_str)
{
	// TODO: Consider zero-byte character at the end of string?
	if (str.size() > maxSize_T_String)
		throw std::runtime_error("string is longer than maxSize_T_String");
	memcpy(t_str.arr, str.c_str(), str.size());
	t_str.nCount = str.size();
}

void fromASN1SCC(const T_String& t_str, std::string& str)
{
	str = std::string((const char*)t_str.arr, (size_t)t_str.nCount);
}
