#ifndef __VW_FILEIO_MEMORYIMAGERESOURCEJPEG_H__
#define __VW_FILEIO_MEMORYIMAGERESOURCEJPEG_H__

#include <vw/FileIO/MemoryImageResource.h>
#include <boost/noncopyable.hpp>

namespace vw {

  class SrcMemoryImageResourceJPEG : public SrcMemoryImageResource, private boost::noncopyable {
      struct Data;
      mutable boost::shared_ptr<Data> m_data;

    public:
      SrcMemoryImageResourceJPEG(boost::shared_array<const uint8> buffer, size_t len);

      virtual void read( ImageBuffer const& buf, BBox2i const& bbox ) const;

      virtual ImageFormat format() const;

      virtual bool has_block_read() const  {return false;}
      virtual bool has_nodata_read() const {return false;}
  };

  class DstMemoryImageResourceJPEG : public DstMemoryImageResource {
      struct Data;
      boost::shared_ptr<Data> m_data;

    public:
      DstMemoryImageResourceJPEG(const ImageFormat& fmt);

      virtual void write( ImageBuffer const& buf, BBox2i const& bbox );
      virtual void flush() {}

      virtual bool has_block_write()  const {return false;}
      virtual bool has_nodata_write() const {return false;}

      virtual const uint8* data() const;
      virtual size_t size() const;
  };

}

#endif
