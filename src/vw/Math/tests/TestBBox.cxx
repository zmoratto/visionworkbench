// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>
#include <test/Helpers.h>
#include <vw/Math/Vector.h>
#include <vw/Math/BBox.h>

using namespace vw;

TEST(BBox, Static) {
  // Default constructor (floating-point type)
  // Hard to know how to test this one....
  BBox<float,2> b1;

  // Default constructor (integer type)
  BBox<uint8,1> b2;
  EXPECT_EQ( 255, b2.min()[0] );
  EXPECT_EQ( 0, b2.max()[0] );
  EXPECT_TRUE( b2.empty() );

  // Two-vector constructor
  BBox2 b3( Vector2(1,2), Vector2(3,4) );
  EXPECT_VECTOR_EQ( Vector2(1,2), b3.min() );
  EXPECT_VECTOR_EQ( Vector2(3,4), b3.max() );

  // Four-number constructor
  BBox2 b4(1,2,1,4);
  EXPECT_VECTOR_EQ( Vector2(1,2), b4.min() );
  EXPECT_VECTOR_EQ( Vector2(2,6), b4.max() );

  // Six-number constructor
  BBox3 b5(1,2,3,1,4,6);
  EXPECT_VECTOR_EQ( Vector3(1,2,3), b5.min() );
  EXPECT_VECTOR_EQ( Vector3(2,6,9), b5.max() );

  // Copy constructor
  BBox3 b6(b5);
  EXPECT_VECTOR_EQ( Vector3(1,2,3), b6.min() );
  EXPECT_VECTOR_EQ( Vector3(2,6,9), b6.max() );

  // Assorted accessors
  EXPECT_VECTOR_EQ( Vector3(1,4,6),   b6.size() );
  EXPECT_VECTOR_EQ( Vector3(1.5,4,6), b6.center() );
  EXPECT_VECTOR_EQ( Vector3(1,2,3),   b6.min() );
  EXPECT_VECTOR_EQ( Vector3(2,6,9),   b6.max() );
  EXPECT_EQ( 1, b6.width() );
  EXPECT_EQ( 4, b6.height() );
  EXPECT_EQ( 6, b6.depth() );
}

TEST(BBox, Dynamic) {
  // Default constructor (floating-point type)
  // Hard to know how to test this one....
  BBox<float> b1;

  // Default constructor (integer type)
  BBox<uint8> b2;
  EXPECT_TRUE( b2.empty() );

  // Two-vector constructor
  BBoxN b3( Vector2(1,2), Vector2(3,4) );
  EXPECT_VECTOR_EQ( Vector2(1,2), b3.min() );
  EXPECT_VECTOR_EQ( Vector2(3,4), b3.max() );

  // Four-number constructor
  BBoxN b4(1,2,1,4);
  EXPECT_VECTOR_EQ( Vector2(1,2), b4.min() );
  EXPECT_VECTOR_EQ( Vector2(2,6), b4.max() );

  // Six-number constructor
  BBoxN b5(1,2,3,1,4,6);
  EXPECT_VECTOR_EQ( Vector3(1,2,3), b5.min() );
  EXPECT_VECTOR_EQ( Vector3(2,6,9), b5.max() );

  // Copy constructor
  BBoxN b6(b5);
  EXPECT_VECTOR_EQ( Vector3(1,2,3), b6.min() );
  EXPECT_VECTOR_EQ( Vector3(2,6,9), b6.max() );

  // Assorted accessors
  EXPECT_VECTOR_EQ( Vector3(1,4,6),   b6.size() );
  EXPECT_VECTOR_EQ( Vector3(1.5,4,6), b6.center() );
  EXPECT_VECTOR_EQ( Vector3(1,2,3),   b6.min() );
  EXPECT_VECTOR_EQ( Vector3(2,6,9),   b6.max() );
  EXPECT_EQ( 1, b6.width() );
  EXPECT_EQ( 4, b6.height() );
  EXPECT_EQ( 6, b6.depth() );
}

TEST(BBox, Ops) {
  BBox2 b4(1,2,1,4);
  BBoxN b5(1,2,1,4);

  // Grow to include a point
  b4.grow( Vector2(0,7) );
  EXPECT_DOUBLE_EQ( 0, b4.min()[0] );
  EXPECT_DOUBLE_EQ( 2, b4.min()[1] );
  EXPECT_DOUBLE_EQ( 2, b4.max()[0] );
  EXPECT_DOUBLE_EQ( 7, b4.max()[1] );

  b5.grow( Vector2(0,7) );
  EXPECT_DOUBLE_EQ( 0, b5.min()[0] );
  EXPECT_DOUBLE_EQ( 2, b5.min()[1] );
  EXPECT_DOUBLE_EQ( 2, b5.max()[0] );
  EXPECT_DOUBLE_EQ( 7, b5.max()[1] );

  // Grow to include a bbox
  b4.grow( BBox2(0,3,3,3) );
  EXPECT_DOUBLE_EQ( 0, b4.min()[0] );
  EXPECT_DOUBLE_EQ( 2, b4.min()[1] );
  EXPECT_DOUBLE_EQ( 3, b4.max()[0] );
  EXPECT_DOUBLE_EQ( 7, b4.max()[1] );

  b5.grow( BBox2(0,3,3,3) );
  EXPECT_DOUBLE_EQ( 0, b5.min()[0] );
  EXPECT_DOUBLE_EQ( 2, b5.min()[1] );
  EXPECT_DOUBLE_EQ( 3, b5.max()[0] );
  EXPECT_DOUBLE_EQ( 7, b5.max()[1] );

  // Crop to bbox
  b4.crop( BBox2(2,0,1,4) );
  EXPECT_DOUBLE_EQ( 2, b4.min()[0] );
  EXPECT_DOUBLE_EQ( 2, b4.min()[1] );
  EXPECT_DOUBLE_EQ( 3, b4.max()[0] );
  EXPECT_DOUBLE_EQ( 4, b4.max()[1] );

  b5.crop( BBox2(2,0,1,4) );
  EXPECT_DOUBLE_EQ( 2, b5.min()[0] );
  EXPECT_DOUBLE_EQ( 2, b5.min()[1] );
  EXPECT_DOUBLE_EQ( 3, b5.max()[0] );
  EXPECT_DOUBLE_EQ( 4, b5.max()[1] );

  // Expand
  b4.expand(2);
  EXPECT_DOUBLE_EQ( 0, b4.min()[0] );
  EXPECT_DOUBLE_EQ( 0, b4.min()[1] );
  EXPECT_DOUBLE_EQ( 5, b4.max()[0] );
  EXPECT_DOUBLE_EQ( 6, b4.max()[1] );

  b5.expand(2);
  EXPECT_DOUBLE_EQ( 0, b5.min()[0] );
  EXPECT_DOUBLE_EQ( 0, b5.min()[1] );
  EXPECT_DOUBLE_EQ( 5, b5.max()[0] );
  EXPECT_DOUBLE_EQ( 6, b5.max()[1] );

  // Contract
  b4.contract(1);
  EXPECT_DOUBLE_EQ( 1, b4.min()[0] );
  EXPECT_DOUBLE_EQ( 1, b4.min()[1] );
  EXPECT_DOUBLE_EQ( 4, b4.max()[0] );
  EXPECT_DOUBLE_EQ( 5, b4.max()[1] );

  b5.contract(1);
  EXPECT_DOUBLE_EQ( 1, b5.min()[0] );
  EXPECT_DOUBLE_EQ( 1, b5.min()[1] );
  EXPECT_DOUBLE_EQ( 4, b5.max()[0] );
  EXPECT_DOUBLE_EQ( 5, b5.max()[1] );

  // Contains points
  EXPECT_TRUE  ( b4.contains ( Vector2(2,3) ));
  EXPECT_FALSE ( b4.contains ( Vector2(0,3) ));
  EXPECT_FALSE ( b4.contains ( Vector2(2,6) ));
  EXPECT_TRUE  ( b4.contains ( b4.min() ));
  EXPECT_FALSE ( b4.contains ( b4.max() ));

  EXPECT_TRUE  ( b5.contains ( Vector2(2,3) ));
  EXPECT_FALSE ( b5.contains ( Vector2(0,3) ));
  EXPECT_FALSE ( b5.contains ( Vector2(2,6) ));
  EXPECT_TRUE  ( b5.contains ( b4.min() ));
  EXPECT_FALSE ( b5.contains ( b4.max() ));

  // Contains bboxes
  EXPECT_TRUE  ( b4.contains ( BBox2(2,2,2,2) ));
  EXPECT_TRUE  ( b4.contains ( BBoxN(2,2,2,2) ));
  EXPECT_FALSE ( b4.contains ( BBox2(0,2,4,2) ));
  EXPECT_FALSE ( b4.contains ( BBoxN(2,0,2,4) ));
  EXPECT_FALSE ( b4.contains ( BBox2(2,2,4,2) ));
  EXPECT_FALSE ( b4.contains ( BBoxN(2,2,4,4) ));
  EXPECT_TRUE  ( b4.contains ( b4 ));

  EXPECT_TRUE  ( b5.contains ( BBox2(2,2,2,2) ));
  EXPECT_TRUE  ( b5.contains ( BBoxN(2,2,2,2) ));
  EXPECT_FALSE ( b5.contains ( BBox2(0,2,4,2) ));
  EXPECT_FALSE ( b5.contains ( BBoxN(2,0,2,4) ));
  EXPECT_FALSE ( b5.contains ( BBox2(2,2,4,2) ));
  EXPECT_FALSE ( b5.contains ( BBoxN(2,2,4,4) ));
  EXPECT_TRUE  ( b5.contains ( b4 ));

  // Intersects
  EXPECT_TRUE  ( b4.intersects ( BBox2(2,2,2,2) ));
  EXPECT_TRUE  ( b4.intersects ( BBoxN(0,2,4,2) ));
  EXPECT_TRUE  ( b4.intersects ( BBox2(2,2,4,4) ));
  EXPECT_FALSE ( b4.intersects ( BBoxN(0,0,1,1) ));
  EXPECT_TRUE  ( b4.intersects ( BBox2(0,0,2,2) ));
  EXPECT_FALSE ( b4.intersects ( BBox2(2,5,2,2) ));

  EXPECT_TRUE  ( b5.intersects ( BBox2(2,2,2,2) ));
  EXPECT_TRUE  ( b5.intersects ( BBoxN(0,2,4,2) ));
  EXPECT_TRUE  ( b5.intersects ( BBox2(2,2,4,4) ));
  EXPECT_FALSE ( b5.intersects ( BBoxN(0,0,1,1) ));
  EXPECT_TRUE  ( b5.intersects ( BBox2(0,0,2,2) ));
  EXPECT_FALSE ( b5.intersects ( BBox2(2,5,2,2) ));

  // Empty
  EXPECT_FALSE( b4.empty());
  b4.min()[0] = b4.max()[0];
  EXPECT_TRUE( b4.empty());

  EXPECT_FALSE( b5.empty());
  b5.min()[0] = b5.max()[0];
  EXPECT_TRUE( b5.empty());
}

TEST(BBox, Math) {
  BBox2 b1(1,2,2,2), b2;
  BBoxN b3(1,2,2,2);

  // Scale-assignment
  b1 *= 2;
  EXPECT_VECTOR_EQ( Vector2(2,4), b1.min() );
  EXPECT_VECTOR_EQ( Vector2(6,8), b1.max() );

  b3 *= 2;
  EXPECT_VECTOR_EQ( Vector2(2,4), b3.min() );
  EXPECT_VECTOR_EQ( Vector2(6,8), b3.max() );

  // Sum-assignment
  b1 += Vector2(2,3);
  EXPECT_VECTOR_EQ( Vector2(4,7), b1.min() );
  EXPECT_VECTOR_EQ( Vector2(8,11), b1.max() );

  b3 += Vector2(2,3);
  EXPECT_VECTOR_EQ( Vector2(4,7), b3.min() );
  EXPECT_VECTOR_EQ( Vector2(8,11), b3.max() );

  // Difference-assignment
  b1 -= Vector2(4,4);
  EXPECT_VECTOR_EQ( Vector2(0,3), b1.min() );
  EXPECT_VECTOR_EQ( Vector2(4,7), b1.max() );

  b3 -= Vector2(4,4);
  EXPECT_VECTOR_EQ( Vector2(0,3), b3.min() );
  EXPECT_VECTOR_EQ( Vector2(4,7), b3.max() );

  // Left scale
  b2 = 2*b1;
  EXPECT_VECTOR_EQ( Vector2(0,6), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(8,14), b2.max() );

  b2 = 2*b3;
  EXPECT_VECTOR_EQ( Vector2(0,6), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(8,14), b2.max() );

  // Right scale
  b2 = b1*3;
  EXPECT_VECTOR_EQ( Vector2(0,9), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(12,21), b2.max() );

  b2 = b3*3;
  EXPECT_VECTOR_EQ( Vector2(0,9), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(12,21), b2.max() );

  // Right quotient
  b2 = b1/2;
  EXPECT_VECTOR_EQ( Vector2(0,1.5), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(2,3.5), b2.max() );

  b2 = b3/2;
  EXPECT_VECTOR_EQ( Vector2(0,1.5), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(2,3.5), b2.max() );

  // Right vector sum
  b2 = b1 + Vector2(1,1);
  EXPECT_VECTOR_EQ( Vector2(1,4), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(5,8), b2.max() );

  b2 = b3 + Vector2(1,1);
  EXPECT_VECTOR_EQ( Vector2(1,4), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(5,8), b2.max() );

  // Left vector sum
  b2 = Vector2(1,1) + b1;
  EXPECT_VECTOR_EQ( Vector2(1,4), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(5,8), b2.max() );

  b2 = Vector2(1,1) + b3;
  EXPECT_VECTOR_EQ( Vector2(1,4), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(5,8), b2.max() );

  // Vector sum
  b2 = b1 - Vector2(1,1);
  EXPECT_VECTOR_EQ( Vector2(-1,2), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(3,6), b2.max() );

  b2 = b3 - Vector2(1,1);
  EXPECT_VECTOR_EQ( Vector2(-1,2), b2.min() );
  EXPECT_VECTOR_EQ( Vector2(3,6), b2.max() );
}

TEST(BBox, Round) {

  // make sure that an integer-truncated bbox always contains the original fp
  // bbox
  const BBox2  af(0,0,2,2);
  const BBox2i ai(0,0,2,2);
  EXPECT_EQ(af, BBox2(ai))  << "fp(int),  no change";
  EXPECT_EQ(ai, BBox2i(af)) << "int(fp),  no change";
  EXPECT_EQ(ai, BBox2i(ai)) << "int(int), no change";
  EXPECT_EQ(af, BBox2(af))  << "fp(fp),   no change";

  const BBox2  bf(-0.8, 2.1);
  const BBox2 bfe(-1, 3);
  const BBox2i bi(-1, 3);
  EXPECT_EQ(bfe, BBox2(bi)) << "fp(int),  no change";
  EXPECT_EQ(bi, BBox2i(bf)) << "int(fp),  round";
  EXPECT_EQ(bi, BBox2i(bi)) << "int(int), no change";
  EXPECT_EQ(bf, BBox2(bf))  << "fp(fp),   no change";

  const BBox2  cf(0,0,2,2);
  const BBox2i ci(0,0,2,2);
  BBox2  cf2;
  BBox2i ci2;

  cf2 = ci;
  EXPECT_EQ(cf, cf2) << " fp = int, no change";
  ci2 = cf;
  EXPECT_EQ(ci, ci2) << "int = fp,  no change";
  ci2 = ci;
  EXPECT_EQ(ci, ci2) << "int = int, no change";
  cf2 = cf;
  EXPECT_EQ(cf, cf2) << " fp = fp,  no change";

  const BBox2  df(-0.8, 2.1);
  const BBox2 dfe(-1, 3);
  const BBox2i di(-1, 3);
  BBox2 df2;
  BBox2i di2;

  df2 = di;
  EXPECT_EQ(dfe, df2) << " fp = int, no change";
  di2 = df;
  EXPECT_EQ(di, di2) << "int = fp,  round";
  di2 = di;
  EXPECT_EQ(di, di2) << "int = int, no change";
  df2 = df;
  EXPECT_EQ(df, df2) << " fp = fp,  no change";

}

TEST(BBox, Grow ) {

  // Evaluates to 2.22e-16
  const double eps = std::numeric_limits<double>::epsilon();

  BBox2 bf;
  bf.grow(Vector2(1,1));
  EXPECT_TRUE( bf.contains( Vector2(1,1) ) );
  EXPECT_VECTOR_EQ( Vector2(1,1), bf.min() );
  EXPECT_VECTOR_NEAR( Vector2(1,1), bf.max(), eps*2 );
  EXPECT_VECTOR_NEAR( Vector2(), bf.size(), eps*2 );

  BBox2i bi;
  bi.grow(Vector2i(1,1));
  EXPECT_TRUE( bi.contains( Vector2i(1,1) ) );
  EXPECT_VECTOR_EQ( Vector2i(1,1), bi.min() );
  EXPECT_VECTOR_EQ( Vector2i(2,2), bi.max() );
  EXPECT_VECTOR_EQ( Vector2i(1,1), bi.size() );

  bf.grow(Vector2(1,2));
  EXPECT_TRUE( bf.contains( Vector2(1,2) ) );
  EXPECT_VECTOR_EQ( Vector2(1,1), bf.min() );
  EXPECT_VECTOR_NEAR( Vector2(1,2), bf.max(), eps*3 );
  EXPECT_VECTOR_NEAR( Vector2(0,1), bf.size(), eps*2 );

  bi.grow(Vector2(1,2));
  EXPECT_TRUE( bi.contains( Vector2i(1,2) ) );
  EXPECT_VECTOR_EQ( Vector2i(1,1), bi.min() );
  EXPECT_VECTOR_EQ( Vector2i(2,3), bi.max() );
  EXPECT_VECTOR_EQ( Vector2i(1,2), bi.size() );

  bf.grow(Vector2());
  EXPECT_TRUE( bf.contains( Vector2() ) );
  EXPECT_VECTOR_EQ( Vector2(0,0), bf.min() );
  EXPECT_VECTOR_NEAR( Vector2(1,2), bf.max(), eps*3 );
  EXPECT_VECTOR_NEAR( Vector2(1,2), bf.size(), eps*3 );

  bi.grow(Vector2i());
  EXPECT_TRUE( bi.contains( Vector2() ) );
  EXPECT_VECTOR_EQ( Vector2i(0,0), bi.min() );
  EXPECT_VECTOR_EQ( Vector2(2,3), bi.max() );
  EXPECT_VECTOR_EQ( Vector2i(2,3), bi.size() );

  // Now for a large jump to make sure the epsilon scaling is still
  // working.
  bf.grow(Vector2(1300,1.6e5));
  EXPECT_TRUE( bf.contains( Vector2(1300,1.6e5) ) );
  EXPECT_VECTOR_EQ( Vector2(0,0), bf.min() );
  EXPECT_VECTOR_NEAR( Vector2(1300,1.6e5), bf.max(), eps*1.6e5 );
  EXPECT_VECTOR_NEAR( Vector2(1300,1.6e5), bf.size(), eps*1.6e5 );
}

TEST(BBox, MinMaxAlternatives ) {

  BBox2 bf;
  bf.grow( Vector2(5,5) );
  ASSERT_TRUE( bf.contains(Vector2(5,5)) );

  EXPECT_LT( bf.inclusive_max()[0], bf.max()[0] );
  EXPECT_LT( bf.inclusive_max()[1], bf.max()[1] );
  EXPECT_LT( bf.inclusive_max()[0], bf.exclusive_max()[0] );
  EXPECT_LT( bf.inclusive_max()[1], bf.exclusive_max()[1] );
  EXPECT_LT( bf.exclusive_min()[0], bf.min()[0] );
  EXPECT_LT( bf.exclusive_min()[1], bf.min()[1] );
  EXPECT_LT( bf.exclusive_min()[0], bf.inclusive_min()[0] );
  EXPECT_LT( bf.exclusive_min()[1], bf.inclusive_min()[1] );

  BBox2i bi;
  bi.grow( Vector2i(5,5) );
  ASSERT_TRUE( bi.contains(Vector2i(5,5)) );

  EXPECT_VECTOR_EQ( Vector2i(5,5), bi.min() );
  EXPECT_VECTOR_EQ( Vector2i(4,4), bi.exclusive_min() );
  EXPECT_VECTOR_EQ( Vector2i(5,5), bi.inclusive_min() );
  EXPECT_VECTOR_EQ( Vector2i(6,6), bi.max() );
  EXPECT_VECTOR_EQ( Vector2i(5,5), bi.inclusive_max() );
  EXPECT_VECTOR_EQ( Vector2i(6,6), bi.exclusive_max() );

}
