/*
Copyright (c) 1998, 1999 Thai Open Source Software Center Ltd
 *
 * The MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef XmlRole_INCLUDED
#define XmlRole_INCLUDED 1

#include "xmltok.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
  XML_ROLE_ERROR = -1,
  XML_ROLE_NONE = 0,
  XML_ROLE_XML_DECL,
  XML_ROLE_INSTANCE_START,
  XML_ROLE_DOCTYPE_NAME,
  XML_ROLE_DOCTYPE_SYSTEM_ID,
  XML_ROLE_DOCTYPE_PUBLIC_ID,
  XML_ROLE_DOCTYPE_CLOSE,
  XML_ROLE_GENERAL_ENTITY_NAME,
  XML_ROLE_PARAM_ENTITY_NAME,
  XML_ROLE_ENTITY_VALUE,
  XML_ROLE_ENTITY_SYSTEM_ID,
  XML_ROLE_ENTITY_PUBLIC_ID,
  XML_ROLE_ENTITY_NOTATION_NAME,
  XML_ROLE_NOTATION_NAME,
  XML_ROLE_NOTATION_SYSTEM_ID,
  XML_ROLE_NOTATION_NO_SYSTEM_ID,
  XML_ROLE_NOTATION_PUBLIC_ID,
  XML_ROLE_ATTRIBUTE_NAME,
  XML_ROLE_ATTRIBUTE_TYPE_CDATA,
  XML_ROLE_ATTRIBUTE_TYPE_ID,
  XML_ROLE_ATTRIBUTE_TYPE_IDREF,
  XML_ROLE_ATTRIBUTE_TYPE_IDREFS,
  XML_ROLE_ATTRIBUTE_TYPE_ENTITY,
  XML_ROLE_ATTRIBUTE_TYPE_ENTITIES,
  XML_ROLE_ATTRIBUTE_TYPE_NMTOKEN,
  XML_ROLE_ATTRIBUTE_TYPE_NMTOKENS,
  XML_ROLE_ATTRIBUTE_ENUM_VALUE,
  XML_ROLE_ATTRIBUTE_NOTATION_VALUE,
  XML_ROLE_ATTLIST_ELEMENT_NAME,
  XML_ROLE_IMPLIED_ATTRIBUTE_VALUE,
  XML_ROLE_REQUIRED_ATTRIBUTE_VALUE,
  XML_ROLE_DEFAULT_ATTRIBUTE_VALUE,
  XML_ROLE_FIXED_ATTRIBUTE_VALUE,
  XML_ROLE_ELEMENT_NAME,
  XML_ROLE_CONTENT_ANY,
  XML_ROLE_CONTENT_EMPTY,
  XML_ROLE_CONTENT_PCDATA,
  XML_ROLE_GROUP_OPEN,
  XML_ROLE_GROUP_CLOSE,
  XML_ROLE_GROUP_CLOSE_REP,
  XML_ROLE_GROUP_CLOSE_OPT,
  XML_ROLE_GROUP_CLOSE_PLUS,
  XML_ROLE_GROUP_CHOICE,
  XML_ROLE_GROUP_SEQUENCE,
  XML_ROLE_CONTENT_ELEMENT,
  XML_ROLE_CONTENT_ELEMENT_REP,
  XML_ROLE_CONTENT_ELEMENT_OPT,
  XML_ROLE_CONTENT_ELEMENT_PLUS,
#ifdef XML_DTD
  XML_ROLE_TEXT_DECL,
  XML_ROLE_IGNORE_SECT,
  XML_ROLE_INNER_PARAM_ENTITY_REF,
#endif /* XML_DTD */
  XML_ROLE_PARAM_ENTITY_REF,
  XML_ROLE_EXTERNAL_GENERAL_ENTITY_NO_NOTATION
};

typedef struct prolog_state {
  int (*handler)(struct prolog_state *state,
	         int tok,
		 const char *ptr,
		 const char *end,
		 const ENCODING *enc);
  unsigned level;
#ifdef XML_DTD
  unsigned includeLevel;
  int documentEntity;
#endif /* XML_DTD */
} PROLOG_STATE;

void XMLTOKAPI XmlPrologStateInit(PROLOG_STATE *);
#ifdef XML_DTD
void XMLTOKAPI XmlPrologStateInitExternalEntity(PROLOG_STATE *);
#endif /* XML_DTD */

#define XmlTokenRole(state, tok, ptr, end, enc) \
 (((state)->handler)(state, tok, ptr, end, enc))

#ifdef __cplusplus
}
#endif

#endif /* not XmlRole_INCLUDED */
