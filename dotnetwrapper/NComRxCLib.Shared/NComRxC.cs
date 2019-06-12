using System;

namespace NComRxCLib
{
    public class NComRxC : IDisposable
    {
        readonly NComRxCSafeHandle handle;

        public NComRxC()
        {
            handle = NComRxCWrapper.CreateDecoder();
        }

        protected virtual void Dispose(bool disposing)
        {
            if (handle != null && !handle.IsInvalid)
                handle.Dispose();
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        public bool UpdateDecoder(byte input)
        {
            bool result = NComRxCWrapper.UpdateDecoder(handle, input);
            return result;
        }

        public double DecodeMeasurement(string measurementName)
        {
            return NComRxCWrapper.DecodeMeasurement(handle, measurementName);
        }

    }

}