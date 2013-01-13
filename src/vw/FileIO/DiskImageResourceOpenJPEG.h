// __BEGIN_LICENSE__
//  Copyright (c) 2006-2012, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

/// \file DiskImageResourceOpenJPEG.h
///
/// Provides support for file formats via OpenJPEG v2
///
#ifndef __VW_FILEIO_DISK_IMAGE_RESOURCE_OPEN_JPEG_H__
#define __VW_FILEIO_DISK_IMAGE_RESOURCE_OPEN_JPEG_H__

#include <boost/shared_ptr.hpp>
#include <vw/FileIO/DiskImageResource.h>

namespace vw {

  struct DiskImageResourceInfoOpenJPEG;

  class DiskImageResourceOpenJPEG : public DiskImageResource {
  public:
    DiskImageResourceOpenJPEG( std::string const& filename,
                               int subsample_factor = 1 ) :
      DiskImageResource( filename ) {
      open( filename, subsample_factor );
    }

    DiskImageResourceOpenJPEG( std::string const& filename,
                               ImageFormat const& format ) :
      DiskImageResource( filename ) {
      create( filename, format );
    }

    virtual ~DiskImageResourceOpenJPEG() {};

    /// Returns the type of disk image resource
    static std::string type_static() { return "OpenJPEG"; }
    virtual std::string type() { return type_static(); }

    /// Access methods
    virtual void read( ImageBuffer const& dest, BBox2i const& bbox ) const;
    virtual void write( ImageBuffer const& dest, BBox2i const& bbox );
    virtual void flush();

    /// Special OpenJPEG settings

    /// Creation methods
    void open( std::string const& filename,
               int subsample_factor = 1 );
    void create( std::string const& filename,
                 ImageFormat const& format );

    static DiskImageResource* construct_open( std::string const& filename );
    static DiskImageResource* construct_create( std::string const& filename,
                                                ImageFormat const& format );

    virtual bool has_block_write()  const { return true; }
    virtual bool has_nodata_write() const { return false; }
    virtual bool has_block_read()   const { return true; }
    virtual bool has_nodata_read()  const { return false; }

    virtual Vector2i block_write_size() const;
    virtual void set_block_write_size( const Vector2i& );
    virtual Vector2i block_read_size() const;

  private:
    boost::shared_ptr<DiskImageResourceInfoOpenJPEG> m_info;
  };

}

#endif//__VW_FILEIO_DISK_IMAGE_RESOURCE_OPEN_JPEG_H__
