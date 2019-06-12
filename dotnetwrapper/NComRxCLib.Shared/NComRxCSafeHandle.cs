using System;
using Microsoft.Win32.SafeHandles;

namespace NComRxCLib
{
    public class NComRxCSafeHandle : SafeHandleZeroOrMinusOneIsInvalid
    {
        public NComRxCSafeHandle() : base(true) { }

        public IntPtr Ptr => this.handle;

        protected override bool ReleaseHandle()
        {
            NComRxCWrapper.DisposeDecoder(handle);
            return true;
        } 

    }
}
