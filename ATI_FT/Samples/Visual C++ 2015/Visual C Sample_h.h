

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 8.00.0603 */
/* at Fri May 26 09:18:41 2017
 */
/* Compiler settings for Visual C Sample.odl:
    Oicf, W1, Zp8, env=Win32 (32b run), target_arch=X86 8.00.0603 
    protocol : dce , ms_ext, c_ext, robust
    error checks: allocation ref bounds_check enum stub_data 
    VC __declspec() decoration level: 
         __declspec(uuid()), __declspec(selectany), __declspec(novtable)
         DECLSPEC_UUID(), MIDL_INTERFACE()
*/
/* @@MIDL_FILE_HEADING(  ) */

#pragma warning( disable: 4049 )  /* more than 64k source lines */


/* verify that the <rpcndr.h> version is high enough to compile this file*/
#ifndef __REQUIRED_RPCNDR_H_VERSION__
#define __REQUIRED_RPCNDR_H_VERSION__ 475
#endif

#include "rpc.h"
#include "rpcndr.h"

#ifndef __RPCNDR_H_VERSION__
#error this stub requires an updated version of <rpcndr.h>
#endif // __RPCNDR_H_VERSION__


#ifndef __Visual_C_Sample_h_h__
#define __Visual_C_Sample_h_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

#ifndef __IVisualCSample_FWD_DEFINED__
#define __IVisualCSample_FWD_DEFINED__
typedef interface IVisualCSample IVisualCSample;

#endif 	/* __IVisualCSample_FWD_DEFINED__ */


#ifndef __VisualCSample_FWD_DEFINED__
#define __VisualCSample_FWD_DEFINED__

#ifdef __cplusplus
typedef class VisualCSample VisualCSample;
#else
typedef struct VisualCSample VisualCSample;
#endif /* __cplusplus */

#endif 	/* __VisualCSample_FWD_DEFINED__ */


#ifdef __cplusplus
extern "C"{
#endif 



#ifndef __VisualCSample_LIBRARY_DEFINED__
#define __VisualCSample_LIBRARY_DEFINED__

/* library VisualCSample */
/* [version][uuid] */ 


DEFINE_GUID(LIBID_VisualCSample,0x4BC1D071,0x3403,0x4633,0x95,0x8D,0x07,0xF4,0xC2,0xF9,0x7E,0x0A);

#ifndef __IVisualCSample_DISPINTERFACE_DEFINED__
#define __IVisualCSample_DISPINTERFACE_DEFINED__

/* dispinterface IVisualCSample */
/* [uuid] */ 


DEFINE_GUID(DIID_IVisualCSample,0x9E2E294D,0x255E,0x4A5D,0xBD,0xF3,0x47,0x1C,0x89,0x51,0x39,0x4A);

#if defined(__cplusplus) && !defined(CINTERFACE)

    MIDL_INTERFACE("9E2E294D-255E-4A5D-BDF3-471C8951394A")
    IVisualCSample : public IDispatch
    {
    };
    
#else 	/* C style interface */

    typedef struct IVisualCSampleVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IVisualCSample * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IVisualCSample * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IVisualCSample * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetTypeInfoCount )( 
            IVisualCSample * This,
            /* [out] */ UINT *pctinfo);
        
        HRESULT ( STDMETHODCALLTYPE *GetTypeInfo )( 
            IVisualCSample * This,
            /* [in] */ UINT iTInfo,
            /* [in] */ LCID lcid,
            /* [out] */ ITypeInfo **ppTInfo);
        
        HRESULT ( STDMETHODCALLTYPE *GetIDsOfNames )( 
            IVisualCSample * This,
            /* [in] */ REFIID riid,
            /* [size_is][in] */ LPOLESTR *rgszNames,
            /* [range][in] */ UINT cNames,
            /* [in] */ LCID lcid,
            /* [size_is][out] */ DISPID *rgDispId);
        
        /* [local] */ HRESULT ( STDMETHODCALLTYPE *Invoke )( 
            IVisualCSample * This,
            /* [annotation][in] */ 
            _In_  DISPID dispIdMember,
            /* [annotation][in] */ 
            _In_  REFIID riid,
            /* [annotation][in] */ 
            _In_  LCID lcid,
            /* [annotation][in] */ 
            _In_  WORD wFlags,
            /* [annotation][out][in] */ 
            _In_  DISPPARAMS *pDispParams,
            /* [annotation][out] */ 
            _Out_opt_  VARIANT *pVarResult,
            /* [annotation][out] */ 
            _Out_opt_  EXCEPINFO *pExcepInfo,
            /* [annotation][out] */ 
            _Out_opt_  UINT *puArgErr);
        
        END_INTERFACE
    } IVisualCSampleVtbl;

    interface IVisualCSample
    {
        CONST_VTBL struct IVisualCSampleVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IVisualCSample_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IVisualCSample_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IVisualCSample_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IVisualCSample_GetTypeInfoCount(This,pctinfo)	\
    ( (This)->lpVtbl -> GetTypeInfoCount(This,pctinfo) ) 

#define IVisualCSample_GetTypeInfo(This,iTInfo,lcid,ppTInfo)	\
    ( (This)->lpVtbl -> GetTypeInfo(This,iTInfo,lcid,ppTInfo) ) 

#define IVisualCSample_GetIDsOfNames(This,riid,rgszNames,cNames,lcid,rgDispId)	\
    ( (This)->lpVtbl -> GetIDsOfNames(This,riid,rgszNames,cNames,lcid,rgDispId) ) 

#define IVisualCSample_Invoke(This,dispIdMember,riid,lcid,wFlags,pDispParams,pVarResult,pExcepInfo,puArgErr)	\
    ( (This)->lpVtbl -> Invoke(This,dispIdMember,riid,lcid,wFlags,pDispParams,pVarResult,pExcepInfo,puArgErr) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */


#endif 	/* __IVisualCSample_DISPINTERFACE_DEFINED__ */


DEFINE_GUID(CLSID_VisualCSample,0xA00BA95E,0x3B28,0x4092,0x82,0x6F,0xA9,0x6B,0x80,0xB3,0xCE,0xDD);

#ifdef __cplusplus

class DECLSPEC_UUID("A00BA95E-3B28-4092-826F-A96B80B3CEDD")
VisualCSample;
#endif
#endif /* __VisualCSample_LIBRARY_DEFINED__ */

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


