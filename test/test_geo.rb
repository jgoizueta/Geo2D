require 'rubygems'
require 'shoulda'
require 'test/unit'
require File.expand_path(File.join(File.dirname(__FILE__),'/../lib/geo2d'))


class TestGeo < Test::Unit::TestCase
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
end

