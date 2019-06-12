using System;
using System.Runtime.InteropServices;

namespace NComRxCLib
{

    internal static class NComRxCWrapper
    {

#if Android
        const string DllName = "libNComRxCLib.so";
#else
        const string DllName = "__Internal";
#endif

        [DllImport(DllName, EntryPoint = "CreateNComRxC")]
        internal static extern NComRxCSafeHandle CreateDecoder();

        [DllImport(DllName, EntryPoint = "DisposeNCoMRxC")]
        internal static extern void DisposeDecoder(IntPtr ptr);

        [DllImport(DllName, EntryPoint = "UpdatePacket")]
        internal static extern bool UpdateDecoder(NComRxCSafeHandle ptr, byte inputChar);

        //Charset ANSI here as we do not need to support unicode characters for measurements names
        [DllImport(DllName, EntryPoint = "GetMeasurement", CharSet = CharSet.Ansi)]
        internal static extern double DecodeMeasurement(NComRxCSafeHandle ptr, string measName);

    }
}
