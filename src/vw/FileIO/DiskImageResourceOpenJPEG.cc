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

#define OPJ_VDM if(::vw::vw_log().is_enabled(vw::VerboseDebugMessage,"fileio")) ::vw::vw_out(vw::VerboseDebugMessage,"fileio")

using namespace vw;

namespace vw {
  struct DiskImageResourceInfoOpenJPEG {
    FILE *fsrc;
    opj_dparameters_t parameters;
    opj_image_t* image;
    opj_stream_t* l_stream;
    opj_codec_t* l_codec;
    opj_codestream_index_t* cstr_index;

    DiskImageResourceInfoOpenJPEG() : fsrc(NULL), image(NULL), l_stream(NULL), l_codec(NULL), cstr_index(NULL) {
      // set decoding parameters to default values
      opj_set_default_decoder_parameters( &parameters );
      parameters.m_verbose = true;
      std::cout << "Cp reduce: " << parameters.cp_reduce << " " << parameters.cp_layer << std::endl;
    }

    ~DiskImageResourceInfoOpenJPEG() {
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
  //(void)client_data;
  VW_OUT( WarningMessage, "fileio") << "DiskImageResourceOpenJPEG Warning: " << msg << std::endl;
}

// Handle OpenJPEG info condition by outputting message text at the
// DebugMessage verbosity level.
static void openjpeg_info_handler( const char* msg, void *client_data ) {
  //(void)client_data;
  VW_OUT( DebugMessage, "fileio") << "DiskImageResourceOpenJPEG Info: " << msg << std::endl;
}

// Handle OpenJPEG error conditions by writing the error and hope the calling
// program checks the return value for the function
static void openjpeg_error_handler( const char* msg, void *client_data ) {
  //(void)client_data;
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

  OPJ_VDM << "Decode format: " << m_info->parameters.decod_format << std::endl;

  m_info->l_codec = opj_create_decompress( OPJ_CODEC_JP2 );

  // Jack in our loggers. Are these thread safe?
  opj_set_info_handler( m_info->l_codec, openjpeg_info_handler, 00);
  opj_set_warning_handler( m_info->l_codec, openjpeg_warning_handler, 00);
  opj_set_error_handler( m_info->l_codec, openjpeg_error_handler, 00);

  OPJ_VDM << "Hit" << std::endl;

  // Setup the decoder using the default parameters
  VW_ASSERT( opj_setup_decoder( m_info->l_codec, &m_info->parameters ),
             vw::ArgumentErr() << "DiskImageResourceOpenJPEG: Failed to setup the decoder.\n" );

  OPJ_VDM << "Hit" << std::endl;

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
            << "  sgnd " << ptr->sgnd << std::endl;
  }


}

void DiskImageResourceOpenJPEG::create( std::string const& filename,
                                            ImageFormat const& format ) {
}

void DiskImageResourceOpenJPEG::read( ImageBuffer const& dest, BBox2i const& bbox ) const {
  VW_ASSERT( int(dest.format.cols)==bbox.width() && int(dest.format.rows)==bbox.height(),
             ArgumentErr() << "DiskImageResourceTIFF (read) Error: Destination buffer has wrong dimensions!" );

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
  return Vector2i(256,256);
}

#undef OPJ_VDM
