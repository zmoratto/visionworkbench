// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <vw/InterestPoint/InterestData.pb.h>
#include <boost/iterator/iterator_traits.hpp>

namespace vw {
namespace ip {

  namespace io {
    inline void ip_to_message( InterestPoint const& ip,
                               IPMessage & message ) {
    }
    inline void message_to_ip( IPMessage const& message,
                               InterestPoint & ip ) {
    }
  }

  /// Writes a gzipped file containing interest points
  template <class IteratorT>
  void write_interest_file( std::string const& file,
                            IteratorT start, IteratorT stop ) {
    typedef typename boost::iterator_category<IteratorT>::iterator_category cat;
    BOOST_STATIC_ASSERT(
      boost::is_convertable<cat,const boost::input_iterator_tag&>::value );

    
  }

  /// Reads a gzipped file containing interest points
  ///
  /// You'll probably want to use a back_inserter to make sure you are
  /// not overfilling your destination
  template <class IteratorT>
  void read_interest_file( IteratorT start,
                           std::string const& file ) {
    typedef typename boost::iterator_category<IteratorT>::iterator_category cat;
    BOOST_STATIC_ASSERT(
      boost::is_convertable<cat,const boost::output_iterator_tag&>::value );
  }

  /// Writes a gzipped file containing matched interest points
  ///
  /// Matched ips are interleaved; they're written in pairs.
  template <class IteratorT1, class IteratorT2>
  void write_match_file( std::string const& file,
                         Iterator start1, Iterator stop1,
                         Iterator start2, Iterator stop2 ) {
    typedef typename boost::iterator_category<IteratorT1>::iterator_category cat1;
    typedef typename boost::iterator_category<IteratorT2>::iterator_category cat2;
    BOOST_STATIC_ASSERT(
      boost::is_convertable<cat1,const boost::input_iterator_tag&>::value );
    BOOST_STATIC_ASSERT(
      boost::is_convertable<cat2,const boost::input_iterator_tag&>::value );
  }

  /// Reads a gzipped file containing matched interest points
  /// 
  /// You'll probably want to use a back_inserter to make sure you are
  /// not overfilling your destination
  template <class IteratorT1, class IteratorT2>
  void read_match_file( IteratorT2 start1, IteratorT2 start2,
                        std::string const& file ) {
    typedef typename boost::iterator_category<IteratorT1>::iterator_category cat1;
    typedef typename boost::iterator_category<IteratorT2>::iterator_category cat2;
    BOOST_STATIC_ASSERT(
      boost::is_convertable<cat1,const boost::output_iterator_tag&>::value );
    BOOST_STATIC_ASSERT(
      boost::is_convertable<cat2,const boost::output_iterator_tag&>::value );
  }

}} // end namespace vw::ip
