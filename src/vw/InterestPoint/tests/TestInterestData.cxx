// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// TestInterestData
#include <gtest/gtest.h>
#include <vw/InterestPoint/InterestIO.h>
#include <test/Helpers.h>

using namespace vw;
using namespace vw::ip;
using namespace vw::test;

class InterestDataTest : public ::testing::Test {
protected:
  InterestDataTest(){}

  virtual void SetUp() {
    ip_list.clear();
    for ( int i = 0; i < 50; i++ ) {
      InterestPoint ip( i*2.1, i*0.9, i+2, i+4, i-7 );
      ip.descriptor = Vector4(i*i,i*7-i,i+99,100-i);
      ip_list.push_back(ip);
    }
  }

  void ip_compare( InterestPoint const& ip1,
                   InterestPoint const& ip2 ) const {
    EXPECT_EQ( ip1.x,     ip2.x );
    EXPECT_EQ( ip1.y,     ip2.y );
    EXPECT_EQ( ip1.scale, ip2.scale );
    EXPECT_EQ( ip1.interest, ip2.interest );
    EXPECT_EQ( ip1.orientation, ip2.orientation );
    EXPECT_EQ( ip1.ix,    ip2.ix );
    EXPECT_EQ( ip1.iy,    ip2.iy );
    EXPECT_VECTOR_FLOAT_EQ( ip1.descriptor, ip2.descriptor );
  }

  std::list<InterestPoint> ip_list;
};

TEST_F( InterestDataTest, VWIP_IOCircle ) {
  UnlinkName ipfile("test.vwip");
  write_interest_file( ipfile, ip_list.begin(),
                       ip_list.end() );
  std::list<InterestPoint> return_list;
  read_interest_file( std::back_inserter( return_list ), ipfile );

  for ( std::list<InterestPoint>::const_iterator ip1 = ip_list.begin(),
          ip2 = return_list.begin();
        ip1 != ip_list.end(); ip1++, ip2++ ) {
    ip_compare( *ip1, *ip2 );
  }
}

TEST_F( InterestDataTest, MATCH_IOCircle ) {
  std::list<InterestPoint> ip_list2;
  std::copy( ip_list.begin(), ip_list.end(),
             std::back_inserter( ip_list2 ) );
  SetUp(); // Remake ip_list

  UnlinkName matchfile("test.match");
  write_match_file( matchfile,
                    ip_list.begin(), ip_list.end(),
                    ip_list2.begin(), ip_list2.end() );
  std::list<InterestPoint> return_list, return_list2;
  read_match_file( std::back_inserter( return_list ),
                   std::back_inserter( return_list2 ),
                   matchfile );
  for ( std::list<InterestPoint>::const_iterator ip1 = ip_list.begin(),
          ip2 = return_list.begin();
        ip1 != ip_list.end(); ip1++, ip2++ ) {
    ip_compare( *ip1, *ip2 );
  }
  for ( std::list<InterestPoint>::const_iterator ip1 = ip_list2.begin(),
          ip2 = return_list2.begin();
        ip1 != ip_list2.end(); ip1++, ip2++ ) {
    ip_compare( *ip1, *ip2 );
  }
}
