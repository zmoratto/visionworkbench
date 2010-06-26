// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <vw/InterestPoint/InterestData.h>
#include <vw/InterestPoint/InterestData.pb.h>

#include <boost/iterator/iterator_traits.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

namespace vw {
namespace ip {

  namespace hidden {
    template <class OStreamT>
    struct WriteIPMessage {
      OStreamT& m_stream;
      WriteIPMessage( OStreamT& stream ) : m_stream(stream) {}

      void operator()( InterestPoint const& ip ) {
        IPMessage proto;
        // Converting to proto buffer
        proto.set_x( ip.x );
        proto.set_y( ip.y );
        proto.set_scale( ip.scale );
        if ( ip.ix != 0 ) proto.set_ix( ip.ix );
        if ( ip.iy != 0 ) proto.set_iy( ip.iy );
        proto.set_polarity( ip.polarity );
        proto.set_interest( ip.interest );
        proto.set_orientation( ip.orientation );

        if ( ip.octave > 0 || ip.scale_lvl > 0 ) {
          proto.set_octave( ip.octave );
          proto.set_scale_lvl( ip.scale_lvl );
        }

        std::copy( ip.begin(), ip.end(),
                   RepeatedFieldBackInserter(proto.mutable_tag()) );

        // Writing to File
        m_stream << vw::int32(proto.ByteSize());
        proto.SerializeToOstream( &m_stream );
      }
    };

    template <class IStreamT>
    struct ReadIPMessage {
      IStreamT& m_stream;
      ReadIPMessage( IStreamT& stream ) : m_stream(stream) {}

      template <class IteratorT>
      void operator()( IteratorT iterator ) {
        // Reading from File
        IPMessage proto;
        vw::int32 bytes;
        m_stream >> bytes;
        char* i = new char[bytes];
        m_stream.read(i,bytes);
        if ( !m_stream.good() )
          return; // filtering stream doesn't seem to raise eof responsibly.
        proto.ParseFromArray(i,bytes);
        delete [] i;

        // Converting to Interest Point
        InterestPoint ip( proto.x(), proto.y() );
        if ( proto.has_scale() )       ip.scale = proto.scale();
        if ( proto.has_ix() )          ip.ix = proto.ix();
        if ( proto.has_iy() )          ip.iy = proto.iy();
        if ( proto.has_orientation() ) ip.orientation = proto.orientation();
        if ( proto.has_interest() )    ip.interest = proto.interest();
        if ( proto.has_polarity() )    ip.polarity = proto.polarity();
        if ( proto.has_octave() )      ip.octave   = proto.octave();
        if ( proto.has_scale_lvl() )   ip.scale_lvl = proto.scale_lvl();
        ip.descriptor =
          VectorProxy<float>( proto.tag_size(),
                               proto.mutable_tag()->mutable_data() );
        *iterator = ip;
      }
    };
  }

  /// Writes a gzipped file containing interest points
  template <class IteratorT>
  void write_interest_file( std::string const& file,
                            IteratorT start, IteratorT stop ) {
    typedef typename boost::iterator_category<IteratorT>::type cat;

    BOOST_STATIC_ASSERT(
      (boost::is_convertible<cat,const std::input_iterator_tag&>::value) );

    namespace io = boost::iostreams;

    std::ofstream ofile(file.c_str(),
                        std::ios_base::out | std::ios_base::binary);
    io::filtering_ostream out;
    out.push(io::gzip_compressor());
    out.push(ofile);

    hidden::WriteIPMessage<io::filtering_ostream> writer(out);
    while (start!=stop) writer( *start++ );

  }

  /// Reads a gzipped file containing interest points
  ///
  /// You'll probably want to use a back_inserter to make sure you are
  /// not overfilling your destination
  template <class IteratorT>
  void read_interest_file( IteratorT start,
                           std::string const& file ) {
    typedef typename boost::iterator_category<IteratorT>::type cat;
    BOOST_STATIC_ASSERT(
      (boost::is_convertible<cat,const std::output_iterator_tag&>::value) );

    namespace io = boost::iostreams;

    std::ifstream ifile(file.c_str(),
                        std::ios_base::in | std::ios_base::binary);
    io::filtering_istream in;
    in.push(io::gzip_decompressor());
    in.push(ifile);

    hidden::ReadIPMessage<io::filtering_istream> reader(in);

    while ( in.good() )
      reader( start++ );
  }

  /// Writes a gzipped file containing matched interest points
  ///
  /// Matched points are written interleaved; pairs at a time.
  template <class IteratorT1, class IteratorT2>
  void write_match_file( std::string const& file,
                         IteratorT1 start1, IteratorT1 stop1,
                         IteratorT2 start2, IteratorT2 stop2 ) {
    typedef typename boost::iterator_category<IteratorT1>::type cat1;
    typedef typename boost::iterator_category<IteratorT2>::type cat2;
    BOOST_STATIC_ASSERT(
      (boost::is_convertible<cat1,const std::input_iterator_tag&>::value) );
    BOOST_STATIC_ASSERT(
      (boost::is_convertible<cat2,const std::input_iterator_tag&>::value) );

    namespace io = boost::iostreams;

    std::ofstream ofile(file.c_str(),
                        std::ios_base::out | std::ios_base::binary);
    io::filtering_ostream out;
    out.push(io::gzip_compressor());
    out.push(ofile);

    hidden::WriteIPMessage<io::filtering_ostream> writer(out);
    while (start1!=stop1 && start2!=stop2) {
      writer( *start1++ );
      writer( *start2++ );
    }
  }

  /// Reads a gzipped file containing matched interest points
  ///
  /// You'll probably want to use a back_inserter to make sure you are
  /// not overfilling your destination
  template <class IteratorT1, class IteratorT2>
  void read_match_file( IteratorT1 start1, IteratorT2 start2,
                        std::string const& file ) {
    typedef typename boost::iterator_category<IteratorT1>::type cat1;
    typedef typename boost::iterator_category<IteratorT2>::type cat2;
    BOOST_STATIC_ASSERT(
      (boost::is_convertible<cat1,const std::output_iterator_tag&>::value) );
    BOOST_STATIC_ASSERT(
      (boost::is_convertible<cat2,const std::output_iterator_tag&>::value) );

    namespace io = boost::iostreams;

    std::ifstream ifile(file.c_str(),
                        std::ios_base::in | std::ios_base::binary);
    io::filtering_istream in;
    in.push(io::gzip_decompressor());
    in.push(ifile);

    hidden::ReadIPMessage<io::filtering_istream> reader(in);

    while ( in.good() ) {
      reader( start1++ );
      reader( start2++ );
    }
  }

}} // end namespace vw::ip
