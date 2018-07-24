#ifndef _ASN1SCC_CONVERSIONS_H_
#define _ASN1SCC_CONVERSIONS_H_

// -----------
// Basic types 
// -----------
#include <string>

// -----------
// ASN1 types 
// -----------
#include <infuse_asn1_types/taste-extended.h>


void toASN1SCC(const std::string& str, T_String& t_str);

void fromASN1SCC(const T_String& t_str, std::string& str);


#endif // _ASN1SCC_CONVERSIONS_H_
