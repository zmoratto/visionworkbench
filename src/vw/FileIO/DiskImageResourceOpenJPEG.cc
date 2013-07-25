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

#include <stdio.h>
#include <openjpeg.h>
#include <opj_config.h>

#include <vw/FileIO/DiskImageResourceOpenJPEG.h>

#define OPJ_VDM if(::vw::vw_log().is_enabled(vw::VerboseDebugMessage,"fileio")) ::vw::vw_out(vw::VerboseDebugMessage,"fileio") << "DiskImageResourceOpenJPEG: "

using namespace vw;

namespace vw {
  struct DiskImageResourceInfoOpenJPEG {
    FILE *fsrc;
    opj_dparameters_t parameters;
    opj_image_t* image;
    opj_stream_t* l_stream;
    opj_codec_t* l_codec;
    opj_codestream_index_t* cstr_index;
    opj_codestream_info_v2_t* codec_info;

    DiskImageResourceInfoOpenJPEG() : fsrc(NULL), image(NULL), l_stream(NULL), l_codec(NULL), cstr_index(NULL), codec_info(NULL) {
      // set decoding parameters to default values
      opj_set_default_decoder_parameters( &parameters );
      parameters.m_verbose = true;
    }

    ~DiskImageResourceInfoOpenJPEG() {
      if ( codec_info )
        opj_destroy_cstr_info( &codec_info );
      if ( l_stream )
        opj_stream_destroy( l_stream );
      if ( fsrc )
        fclose( fsrc );
      if ( l_codec )
        opj_destroy_codec( l_codec );
      if ( image )
        opj_image_destroy( image );
    }
  };
}

// Handle OpenJPEG warning condition by outputting message text at the
// Warning verbosity level.
static void openjpeg_warning_handler( const char* msg, void *client_data ) {
  (void)client_data;
  VW_OUT( WarningMessage, "fileio") << "DiskImageResourceOpenJPEG Warning: " << msg << std::endl;
}

// Handle OpenJPEG info condition by outputting message text at the
// DebugMessage verbosity level.
static void openjpeg_info_handler( const char* msg, void *client_data ) {
  (void)client_data;
  VW_OUT( DebugMessage, "fileio") << "DiskImageResourceOpenJPEG Info: " << msg << std::endl;
}

// Handle OpenJPEG error conditions by writing the error and hope the calling
// program checks the return value for the function
static void openjpeg_error_handler( const char* msg, void *client_data ) {
  (void)client_data;
  VW_OUT( ErrorMessage, "fileio") << "DiskImageResourceOpenJPEG Error: " << msg << std::endl;
}

void vw::DiskImageResourceOpenJPEG::open( std::string const& filename,
                                          int subsample_factor ) {

  m_info.reset( new DiskImageResourceInfoOpenJPEG() );

  OPJ_VDM << "Trying to open \"" << filename << "\"" << std::endl;

  // Read the input file and put it in memory
  m_info->fsrc = fopen( filename.c_str(), "rb" );
  if ( !m_info->fsrc )
    vw_throw( vw::ArgumentErr() << "DiskImageResourceOpenJPEG: Failed to open \"" << filename << "\" for reading!");

  // Mutexing access to the stream .. might be or ticket for parallelization
  // the 1 means read only
  m_info->l_stream = opj_stream_create_default_file_stream(m_info->fsrc,1);
  VW_ASSERT( m_info->l_stream,
             ArgumentErr() << "Failed to create stream from file\n" );

  OPJ_VDM << "Decode format: " << m_info->parameters.decod_format << std::endl;

  //m_info->l_codec = opj_create_decompress( OPJ_CODEC_JP2 );
  m_info->l_codec = opj_create_decompress( OPJ_CODEC_J2K );
  VW_ASSERT( m_info->l_codec,
             ArgumentErr() << "Failed to create codec.\n" );

  // Jack in our loggers. Are these thread safe?
  opj_set_info_handler( m_info->l_codec, openjpeg_info_handler, 00);
  opj_set_warning_handler( m_info->l_codec, openjpeg_warning_handler, 00);
  opj_set_error_handler( m_info->l_codec, openjpeg_error_handler, 00);

  // Setup the decoder using the default parameters
  VW_ASSERT( opj_setup_decoder( m_info->l_codec, &m_info->parameters ),
             vw::ArgumentErr() << "DiskImageResourceOpenJPEG: Failed to setup the decoder.\n" );

  // Read the main header of the codestream and if necessary the JP2 boxes
  VW_ASSERT( opj_read_header( m_info->l_stream, m_info->l_codec, &(m_info->image) ),
             vw::ArgumentErr() << "DiskImageResourceOpenJPEG: Failed to read the header.\n" );

  // Debug drop some information about the JP2
  OPJ_VDM << "Offset of reference grid: " << m_info->image->x0 << " " << m_info->image->y0 << std::endl;
  OPJ_VDM << "Size of reference grid:   " << m_info->image->x1 << " " << m_info->image->y1 << std::endl;
  OPJ_VDM << "Number of image components: " << m_info->image->numcomps << std::endl;
  for ( uint32 i = 0; i < m_info->image->numcomps; i++ ) {
    opj_image_comp_t* ptr = m_info->image->comps + i;
    OPJ_VDM << "Component " << i << std::endl
            << "  dxdy " << ptr->dx << " " << ptr->dy << std::endl
            << "  wh   " << ptr->w << " " << ptr->h << std::endl
            << "  x0y0 " << ptr->x0 << " " << ptr->y0 << std::endl
            << "  prec " << ptr->prec << std::endl
            << "  bpp  " << ptr->bpp << std::endl
            << "  sgnd " << ptr->sgnd << std::endl
            << "  data " << ptr->data << std::endl;
  }

  // Read codec stream information to see if it tells me tile information
  m_info->codec_info = opj_get_cstr_info( m_info->l_codec );
  OPJ_VDM << "Tile origin: " << m_info->codec_info->tx0 << " "
          << m_info->codec_info->ty0 << std::endl;
  OPJ_VDM << "Tile size:   " << m_info->codec_info->tdx << " "
          << m_info->codec_info->tdy << std::endl;
  OPJ_VDM << "Number of tiles: " << m_info->codec_info->tw << " "
          << m_info->codec_info->th << std::endl;
  OPJ_VDM << "Number of components: " << m_info->codec_info->nbcomps
          << std::endl;

  // Tell VW about the image format
  m_format.cols = m_info->image->x1;
  m_format.rows = m_info->image->y1;
  m_format.planes = 1;

  // Need to try and support multispectral in the future.

  if ( m_info->image->comps->sgnd ) {
    m_format.channel_type = VW_CHANNEL_INT16;
  } else {
    m_format.channel_type = VW_CHANNEL_UINT16;
  }
  m_format.pixel_format = VW_PIXEL_GRAY;
}

void DiskImageResourceOpenJPEG::create( std::string const& filename,
                                        ImageFormat const& format ) {
}

void DiskImageResourceOpenJPEG::read( ImageBuffer const& dest, BBox2i const& bbox ) const {
  VW_ASSERT( int(dest.format.cols)==bbox.width() && int(dest.format.rows)==bbox.height(),
             ArgumentErr() << "DiskImageResourceTIFF (read) Error: Destination buffer has wrong dimensions!" );

  Vector2i tile_size = block_read_size();

  opj_image_t* tile_image_data = NULL;
  VW_ASSERT( opj_read_header( m_info->l_codec, m_info->l_stream, &tile_image_data ),
             IOErr() << "Failed to reparse image header." );

  if ( !(bbox.min().x() % tile_size.x()) &&
       !(bbox.min().y() % tile_size.y()) ) {
    // See if they are requesting on a tile boundary
    Vector2i index = elem_quot(bbox.min(), tile_size);

    if ( bbox.size() == tile_size ) {
      // Call for decoding of a single tile!
      OPJ_VDM << "Decoding tile at " << bbox << ": Which should be tile: " << index.x() + index.y() * m_info->codec_info->tw << std::endl;
      VW_ASSERT( opj_get_decoded_tile( m_info->l_codec, m_info->l_stream, tile_image_data,
                                       index.x() + index.y() * m_info->codec_info->tw ),
                 IOErr() << "Failed to decoded tile index: " << index.x() + index.y() * m_info->codec_info->tw );
      OPJ_VDM << "Finished!" << std::endl << std::flush;

      // Give the image buffer ownership of the data
      ImageBuffer src_buf;
      src_buf.format = m_format;
      src_buf.format.cols = bbox.width();
      src_buf.format.rows = bbox.height();
      src_buf.data = tile_image_data->comps[0].data;
      src_buf.cstride = 4; // Always 4 bytes?
      src_buf.rstride = src_buf.cstride * tile_image_data->comps[0].w;
      src_buf.pstride = src_buf.rstride * tile_image_data->comps[0].h;
      convert( dest, src_buf, m_rescale);

      opj_image_destroy( tile_image_data );
      return;
    }
  }

  // They are requesting something odd ... no worries just a different api call.
  vw_throw( vw::NoImplErr() << "DiskImageResourceOpenJPEG: Failed to handle non tile read request" );

  return;
}

void DiskImageResourceOpenJPEG::write( ImageBuffer const& src, BBox2i const& bbox ) {

}

void DiskImageResourceOpenJPEG::flush() {

}

DiskImageResource* vw::DiskImageResourceOpenJPEG::construct_open( std::string const& filename ) {
  return new DiskImageResourceOpenJPEG( filename );
}

DiskImageResource* vw::DiskImageResourceOpenJPEG::construct_create( std::string const& filename,
                                                                    ImageFormat const& format ) {
  return new DiskImageResourceOpenJPEG( filename, format );
}

Vector2i DiskImageResourceOpenJPEG::block_write_size() const {
  return Vector2i(256,256);
}

void DiskImageResourceOpenJPEG::set_block_write_size( const Vector2i& ) {

}

Vector2i DiskImageResourceOpenJPEG::block_read_size() const {
  VW_ASSERT( m_info, IOErr() << "DiskImageResourceOpenJPEG: File not opened.\n" );
  VW_ASSERT( m_info->codec_info,
             IOErr() << "DiskImageResourceOpenJPEG: Codec information not loaded.\n" );
  return Vector2i(m_info->codec_info->tdx, m_info->codec_info->tdy);
}

#undef OPJ_VDM
