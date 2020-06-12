/*
RegAccess.h
contains functions the Visual C sample uses to access the registry
*/

#ifndef REGACCESS_H
#define REGACCESS_H

/*
SetKey
Opens and sets a value to a specified string. puts it in key
	HKEY_CURRENT_USER/ATIDAQFTC.
arguments:  valueName - the name of the value to set
			data - the data to set the value to
returns: 0 if success, error code from creating or setting the
	value otherwise
*/
LONG SetRegValue( LPCTSTR valueName, LPCTSTR data);


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
LONG GetRegValue( LPCTSTR valueName, LPCTSTR data, DWORD dataSize);

/*
DeleteRegValue
removes value from the registry key HKEY_CURRENT_USER/ATIDAQFTC
arguments:  valueName - the name of the value to delete
returns: 0 if successful, error code from attempting to delete the
	value otherwise
*/
LONG DeleteRegValue( LPCTSTR valueName );
#endif