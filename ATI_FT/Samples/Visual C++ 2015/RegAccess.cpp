/*
RegAccess.cpp
implementation of registry access functions
*/

#include "StdAfx.h"
#include <windows.h>
#include "RegAccess.h"

/*
SetKey
Opens and sets a value to a specified string. puts it in key
	HKEY_CURRENT_USER/ATIDAQFTC.
arguments:  valueName - the name of the value to set
			data - the data to set the value to
returns: 0 if success, error code from creating or setting the
	value otherwise
*/
LONG SetRegValue( LPCTSTR valueName, LPCTSTR data)
{
	HKEY hkResult; /*handle to the key*/
	LONG result; /*result of opening and setting the key*/
	result = RegCreateKeyEx( HKEY_CURRENT_USER, "ATIDAQFTC", 0, "", 
		REG_OPTION_NON_VOLATILE, KEY_WRITE, NULL, &hkResult, NULL);
	if ( ERROR_SUCCESS != result)
	{		
		return result;
	}		
	result = RegSetValueEx( hkResult, valueName, 0, REG_SZ, (BYTE *)data, 
		strlen(data) + 1);	
	RegCloseKey( hkResult );	
	return (ERROR_SUCCESS == result) ? 0 : result;
	
}

/*
GetRegValue
Gets string value from registry key HKEY_CURRENT_USER/ATIDAQFTC
arguments:	valueName - the name of the value to get
			data - pointer to store value in, must be 
						large enough to accommodate the
						value from the registry
			dataSize - the size of data
returns: 0 if successful, error code from opening or reading the
	value otherwise
*/
LONG GetRegValue( LPCTSTR valueName, LPCTSTR data, DWORD dataSize)
{
	HKEY keyHandle; /* the handle to the HKEY_CURRENT_USER/ATIDAQFTC
						key*/
	LONG result; /* the result of opening the key and reading the 
					value */
	result = RegOpenKeyEx(HKEY_CURRENT_USER, "ATIDAQFTC", 0, 
		KEY_READ, &keyHandle);
	if ( ERROR_SUCCESS != result)
	{
		return result;
	}
	DWORD type;
	result = RegQueryValueEx( keyHandle, valueName, 0, &type, (BYTE *)data, &dataSize); 
	RegCloseKey( keyHandle);
	return ( ERROR_SUCCESS == result ) ? 0 : result;

}


/*
DeleteRegValue
removes value from the registry key HKEY_CURRENT_USER/ATIDAQFTC
arguments:  valueName - the name of the value to delete
returns: 0 if successful, error code from attempting to delete the
	value otherwise
*/
LONG DeleteRegValue( LPCTSTR valueName )
{
	HKEY regKey; /* the handle to the registry key */
	LONG result; /* result of opening and deleting value */
	result = RegOpenKeyEx(HKEY_CURRENT_USER, "ATIDAQFTC", 0, 
		KEY_ALL_ACCESS, &regKey); 
	if ( ERROR_SUCCESS != result )
	{
		return result;
	}
	result = RegDeleteValue( regKey, valueName );
	return (ERROR_SUCCESS == result) ? 0 : result;	
}