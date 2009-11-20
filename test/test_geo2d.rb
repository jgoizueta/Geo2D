require 'helper'

class TestGeo2d < Test::Unit::TestCase
  include Geo2D
  context "A Vector" do
    setup do
      @vector = Vector.new(10,20)
    end
    should "be constructible by components" do
      assert_equal @vector, Vector(10,20)
    end
    should "be constructible by an array" do
      assert_equal @vector, Vector([10,20])
    end
    should "be constructible by a vector" do
      assert_equal @vector, Vector(@vector)
    end
    should "be constructible by a hash" do
      assert_equal @vector, Vector(:x=>10, :y=>20)
    end
    should "be not constructible by any number of components other than 2" do  
      assert_raise(ArgumentError){Vector(10,20,30)}
      assert_raise(ArgumentError){Vector(10)}
      assert_raise(ArgumentError){Vector([10,20,30])}
      assert_raise(ArgumentError){Vector([10])}
    end
    should "have a modulus" do  
      assert_equal Math.hypot(10,20), @vector.modulus
    end
    should "have an argument" do    
      assert_equal Math.atan2(20,10), @vector.argument
    end
    should "have a conmutative scalar product" do
      assert_equal Vector.new(20,40), 2*@vector
      assert_equal Vector.new(20,40), @vector*2
    end
    should "have a conmutative dot product" do
      assert_equal 70, Vector(10,20)*Vector(3,2)
      assert_equal 70, Vector(3,2)*Vector(10,20)
    end
    should "have equality and inequality operators" do
      assert @vector==Vector(10,20)
      assert @vector!=Vector(20,10)
    end
    should "be splittable" do
      assert [10,20] == @vector.split
      assert [10,20] == @vector.to_a
    end
  end
  
  context "Given a line segment" do
    
    setup do
      @seg = LineSegment([10,20],[78,57])
    end
    
    context "And some points" do

      setup do
        @pnts = [[30,40],[10,20],[78,57],[0,0],[-1,10],[20,30],[0.5*(10+20),0.5*(78+57)]].map{|x,y| Point(x,y)}
      end
    
      should  "reference the points in the line consistently" do        
        seg_max = @seg.points.map{|p| p.length}.max
        @pnts.each do |pnt|
          l,d = @seg.locate_point(pnt)
          pnt2 = @seg.interpolate_point(l,d)
          tolerance = [pnt.modulus, seg_max].max * Float::EPSILON * 2
          assert (pnt2-pnt).length < tolerance, "Point #{pnt} yields #{pnt2} [#{(pnt2-pnt).length}  / #{tolerance}]"
        end
      end
      
    end
  end

  context "Given a line string" do
    
    setup do
      @seg = Line([10,20],[78,57],[30,50],[110,60],[100,30])
    end
    
    context "And some points" do

      setup do
        @pnts = [[30,40],[10,20],[78,57],[0,0],[-1,10],[20,30],[0.5*(10+20),0.5*(78+57)]].map{|x,y| Point(x,y)}
      end
    
      should  "reference the points in the line consistently" do        
        seg_max = @seg.points.map{|p| p.length}.max
        @pnts.each do |pnt|
          l,d = @seg.locate_point(pnt)
          pnt2 = @seg.interpolate_point(l,d)
          tolerance = [pnt.modulus, seg_max].max * Float::EPSILON * 2
          assert (pnt2-pnt).length < tolerance, "Point #{pnt} yields #{pnt2} [#{(pnt2-pnt).length}  / #{tolerance}]"
        end
      end
      
    end
  end
  
end

