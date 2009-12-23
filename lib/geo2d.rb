require 'flt/math'
require 'flt/tolerance'

# Planar geometry of points and line-strings
module Geo2D

  class Context
    def initialize(*options)
      
      @num_class = Float
      @tolerance = Flt::Tolerance(2, :big_epsilon)

      options.dup.each do |option|
        case option
        when Class
          if option.ancestors.include?(Numeric)
            @num_class = options.delete(option)
          end
        when Flt::Tolerance
          @tolerance = options.delete(option)
        when Context
          copy_from options.delete(option)
        end
      end
      
      if options.size>1 || (options.first && !options.first.kind_of?(Hash))
        raise ArgumentError, "Invalid parameters for Geo2D context."
      end
      
      @num_context = @num_class.context
      
      assign options.first
      
    end

    def assign(options)
      if options
        @num_class = options.delete(:num_class) if options.has_key?(:num_class)
        @tolerance = options.delete(:tolerance) if options.has_key?(:tolerance)
        @num_context = options.delete(:num_context) if options.has_key?(:num_context)
        @num_context.assign options
      end
    end
    
    attr_accessor :num_class, :tolerance, :num_context
   
    # Constructor for the associated numeric class
    def Num(*args)
      @num_class.context.Num(*args)
    end


    # Copy the state from other Context object.
    def copy_from(other)
      @num_class = other.num_class
      @tolerance = other.tolerance
      @num_context = other.num_context
    end

    def dup
      self.class.new(self)
    end


    def to_s
      inspect
    end

    def inspect      
      class_name = self.class.to_s.split('::').last
      # "<#{class_name}:\n" +
      # instance_variables.map { |v| "  #{v}: #{eval(v).inspect}"}.join("\n") +
      # ">\n"
    end


    private


  end # Context
  
  
  DefaultContext = Context.new
  
  def self.Context(*args)
    self::Context.new(self::DefaultContext, *args)
  end
  
  # Define a context by passing either of:
  # * A Context object (of the same type)
  # * A hash of options (or nothing) to alter a copy of the current context.
  # * A Context object and a hash of options to alter a copy of it
  def self.define_context(*options)
    context = options.shift if options.first.instance_of?(self::Context)
    if context && options.empty?
      context
    else
      context ||= self.context
      self.Context(context, *options)
    end
  end

  # This makes the class define context accesible to instance methods
  def define_context(*options)
    self.class.define_context(*options)
  end
  private :define_context

  # The current context (thread-local).
  # If arguments are passed they are interpreted as in Geo2D.define_context() to change
  # the current context.
  # If a block is given, this method is a synonym for Geo2D.local_context().
  def self.context(*args, &blk)
    if blk
      # setup a local context
      local_context(*args, &blk)
    elsif args.empty?
      # return the current context
      ctxt = _context
      self._context = ctxt = self::DefaultContext.dup if ctxt.nil?
      ctxt
    else
      # change the current context
      self.context = define_context(*args)
    end
  end

  # Change the current context (thread-local).
  def self.context=(c)
    self._context = c.dup
  end

  # Defines a scope with a local context. A context can be passed which will be
  # set a the current context for the scope; also a hash can be passed with
  # options to apply to the local scope.
  # Changes done to the current context are reversed when the scope is exited.
  def self.local_context(*args)
    begin
      keep = self.context # use this so _context is initialized if necessary
      self.context = define_context(*args) # this dups the assigned context
      result = yield _context
    ensure
      self._context = keep
      result
    end
  end

  class <<self
    # This is the thread-local context storage low level interface
    protected
    def _context #:nodoc:
      Thread.current[:Geo2D_context]
    end
    def _context=(c) #:nodoc:
      Thread.current[:Geo2D_context] = c
    end
  end
  
  module ContextHelper
    protected
    def context
      Geo2D.context
    end
    def Num(*args)
      context.Num(*args)
    end
    
    # Computes the passed block with the proper numeric context
    #   compute{self.x+self.y}
    def compute(&blk)
      Geo2D.context.num_context.eval(&blk)
    end
    
    # like compute, but self points to the context, so context/math functions can be used directly;
    # self is passed as a parameter to the block in case it is needed:
    #   math{|this| hypot(this.x, this.y)}    
    def math(&blk)
      Geo2D.context.num_context.math(self, &blk)
    end
    
    def tolerance
      context.tolerance
    end
  end

# Problems Float.context is singleton; cannot dup
#     compute block cannot access self  . idea: inject variable this=outer self (math should accept paramters to pass to block)
#  def compute(&blk) Geo2D.context.num_context.math(self, &blk); end
#  ... *compute{\this| [this.x*2, this.y*2]}
  
  # Planar vectors; used also to represent points of the plane
  class Vector
    
    include ContextHelper
    
    def initialize(x=0, y=0)
      @x = Num(x)
      @y = Num(y)
    end

    attr_accessor :x, :y

    def modulus
      math{|this| hypot(this.x, this.y)}
    end

    def length
      modulus
    end

    def argument
      math{|this| atan2(this.y, this.x)}
    end

    def +(other)
      other = Geo2D.Vector(other)
      # Vector.new(*compute{|this| [this.x+other.x, this.y+other.y]})
      Vector.new(*compute{[self.x+other.x, self.y+other.y]})
    end

    def -(other)
      other = Geo2D.Vector(other)
      Vector.new(*compute{[self.x-other.x, self.y-other.y]})
    end

    def *(scalar_or_vector)
      if Numeric===scalar_or_vector
        # scalar product
        Vector.new(*math{|this| [scalar_or_vector*this.x, scalar_or_vector*this.y]})
      else
        # dot product
        other = Geo2D.Vector(scalar_or_vector)
        compute(){self.x*other.x + self.y*other.y}
      end
    end

    def /(scalar)
      # self * 1.0/scalar
      Vector.new(*compute{[self.x/scalar, self.y/scalar]})
    end

    # z coordinate of cross product
    def cross_z(other)
      compute{self.x*other.y - other.x*self.y}
    end

    def dot(other)
      compute{self.x*other.x + self.y*other.y}
    end

    def ==(other)
      tolerance.eq?(self.x, other.x) && tolerance.eq?(self.y, other.y)
    end

    def to_a
      [self.x, self.y]
    end

    def to_s
      "(#{self.x}, #{self.y})"
    end

    def split
      to_a
    end

    # unitary vector in the direction of self
    def unitary
      self / self.modulus
    end

    # vector rotated 90 degrees counter-clockwise
    def ortho
      Vector.new(-self.y, self.x)
    end

    # angle between two vectors
    def angle_to(other)
      math{|this| atan2(this.cross_z(other), this.dot(other))}
    end

    def aligned_with?(other)
      tolerance.zero?(cross_z)
    end

    # multiply by matrix [[a11, a12], [a21, a22]]
    def transform(*t)
      a11, a12, a21, a22 = t.flatten.map{|v| Num(v)}
      x, y = self.x, self.y
      Vector.new(*compute{[a11*x + a12*y, a21*x + a22*y]})
    end
    
    # vector rotation (center at origin); for a general rotation use Geo2D.rotation
    def rotate(angle)
      transform(*Geo2D.rotation_transform(angle))
    end

    # Apply arbitrary transformation (passed as a Proc or as a block)
    def apply(prc, &blk)
      prc ||= blk
      prc[self]
    end

    def bounds
      [x,y,x,y]
    end

    def coerce(scalar)
      if scalar.kind_of?(Numeric)
        [self, scalar]
      else
        raise ArgumentError, "Vector: cannot coerce #{scalar.class}"
      end
    end

  end

  def distance_to(other)
    if other.kind_of?(Vector)
      (other-self).modulus
    else
      other.distance_to?(self)
    end
  end

  module_function

  # Vector constructor
  def Vector(*args)
    case args.size
    when 2
      x, y = args
    when 1
      arg = args.first
      if arg.is_a?(Vector)
        return arg
      elsif arg.kind_of?(Array) && arg.size==2
        x, y = arg
      elsif arg.kind_of?(Hash)
        if arg.has_key?(:x) && arg.has_key?(:y)
          x, y = arg[:x], arg[:y]
        end
      else
        if arg.respond_to?(:x) && arg.respond_to?(:y)
          x, y = arg.x, arg.y
        else
          raise ArgumentError,"Invalid point definition"
        end
      end
    else
      raise ArgumentError,"Invalid number of parameters for a point"
    end
    Vector.new(x,y)
  end

  # Line segment between two points (defined by Vectors)
  class LineSegment

    include ContextHelper

    def initialize(p1, p2)
      @start = p1
      @end = p2
      raise ArgumentError,"Degenerate LineSegment" if p1==p2
    end

    attr_reader :start, :end

    def points
      [@start, @end]
    end

    def n_points
      2
    end

    def vector
      @vector ||= (@end-@start)
    end

    def length
      @length ||= vector.modulus
    end

    def angle
      vector.argument
    end

    def angle_at(parallel_distance)
      angle
    end

    def aligned_with?(point)
      vector.aligned_width?(point-@start)
    end

    def contains?(point)
      if self.aligned_with?(point)
        l,d,r = self.locate_point(point)
        l>=0 && l<=self.length # => d==0
      else
        false
      end
    end

    def direction
      @u ||= vector.unitary
    end

    # Returns the position in the segment (distance from the start node along the line) of the nearest line point
    # to the point (point projected on the line) and the perpendicular separation of the point from the line (the
    # distance from the point to the line, but signed: negative when the point is on the right side of the line.
    # If the last parameter is true, the resulting point is forced to lie in the segment (so the distance along
    # the line is between 0 and the segment's length) and the second result is the distance from the point to the
    # segment (i.e. to the closest end of the segment if the projected point lies out of the segment)
    # The third returned value is a rotation that must be applied to the perpendicular vector; it is nonzero
    # only when the last parameter is true and the point is closer to a segment end than to any other segment point.
    def locate_point(point, corrected=false)
      point = Geo2D.Vector(point)
      v = point - @start
      l = v.dot(direction)
      d = direction.cross_z(v) # == (v-l*direction).length == v.length*Math.sin(v.angle_to(direction))
      rotation = 0

      if corrected
        math do |this|
          # rotation = atan2(-(l.modulo(length))*d.sign, d.abs)        
          if l<0
            rotation = atan2(d < 0 ? l : -l, d.abs)
            l = 0
            d = d/cos(rotation) # d.sign*(point-@start).length
          elsif l>this.length
            l -= this.length
            rotation =atan2(d < 0 ? l : -l, d.abs)
            l = this.length
            d = d/cos(rotation) # d = d.sign*(point-@end).length
          end
        end
      end

      [l, d, rotation]
    end

    # Computes the position of a point in the line given the distance along the line from the starting node.
    # If a second parameter is passed it indicates the separation of the computed point in the direction
    # perpendicular to the line; the point is on the left side of the line if the separation is > 0.
    # The third parameter is a rotation applied to the vector from the line to the point.
    def interpolate_point(parallel_distance, separation=0, rotation=0)
      p = @start + self.direction*parallel_distance
      unless separation==0
        d = direction.ortho*separation
        d = d.rotate(rotation) unless rotation==0
        p += d
      end
      p
    end

    # Distance from the segment to a point
    def distance_to(point)
      locate_point(point, true)[1].abs
    end

    # Distance from the line that contains the segment to the point
    def line_distance_to(point)
      locate_point(point, false)[1].abs
    end

    def length_to(point)
      locate_point(point, true).first
    end

    # multiply by matrix [[a11, a12], [a21, a22]]
    def transform(*t)
      LineSegment.new(@start.transform(*t), @end = @end.transform(*t))
    end

    # Apply arbitrary transformation (passed as a Proc or as a block)
    def apply(prc, &blk)
      prc ||= blk
      LineSegment.new(prc[@start], prc[@end])
    end

    # Returns the side of the line that contains the segment in which the point lies:
    # * +1 the point is to the left of the line (as seen from the orientation of the segment)
    # * -1 is in the right side
    # *  0 the point is on the line
    def side_of(point)
      v = vector.cross_z(point-@start)
      v < 0 ? -1 : (v > 0 ? +1 : 0)
    end

    def bounds
      xmn, xmx = [@start.x, @end.x].sort
      ymn, ymx = [@start.y, @end.y].sort
      [xmn, ymn, xmx, ymx]
    end

  end # LineSegment

  class LineString
    
    include ContextHelper

    def initialize(*vertices)
      @vertices = vertices

      to_remove = []
      prev = nil
      @vertices.each_with_index do |v, i|
        to_remove << i if prev && prev==v
        prev = v
      end
      to_remove.each do |i|
        @vertices.delete_at i
      end

    end

    def start
      @vertices.first
    end

    def end
      @vertices.last
    end

    def length
      @length ||= total_length
    end

    def n_points
      @vertices.size
    end
    def points
      @vertices
    end
    def each_point
      @vertices.each do |v|
        yield v
      end
    end

    def n_segments
      [n_points - 1,0].max
    end

    def segments
      (0...n_segments).to_a.map{|i| segment(i)}
    end

    def each_segment
      (0...n_segments).each do |i|
        yield segment(i)
      end
    end

    def segment(i)
      raise ArgumentError, "Invalid segment index #{i}" unless i>=0 && i<n_segments
      LineSegment.new(@vertices[i],@vertices[i+1])
    end

    def distance_to(point)
      locate_point(point, true)[1].abs
    end

    def length_to(point)
      locate_point(point, true).first
    end

    # Return the position of a point in relation to the line: parallalel distance along the line,
    # separation and rotation of the separation (from the perpendicular.)
    # Parallalel distance is in [0,length].
    # If we have l,d,r = line.locate_point(point), then the closest line point to point is:
    # line.interpolate_point(l) and line.interpolate_point(l,d,r) is point; d.abs is the distance from line to point;
    # point is on the left of line if d>0; on the right if d<0 and on the line if d==0.
    # If we want to align a text with the line at point, we would use the angle line.angle_at(l,r)
    def locate_point(point)
      best = nil

      compute do
        total_l = Num(0)
        (0...n_segments).each do |i|

          seg = segment(i)
          seg_l = seg.length

          l,d,rotation = seg.locate_point(point, true)

          if best.nil? || d.abs<best[1].abs
            best = [total_l+l, d, rotation]
          end

          total_l += seg_l
        end
      end

      best
    end

    # Compute a point position given the relative position in the line as returned by locate_point().
    def interpolate_point(parallel_distance, separation=0, rotation=0)
      # separation>0 => left side of line in direction of travel
      i, l = segment_position_of(parallel_distance)
      segment(i).interpolate_point(l, separation, rotation)
    end

    # Angle of the line at a point (in radians, from the X axis, counter-clockwise.)
    # The rotation parameter is added to this angle, if the rotation obtained from locate_point is used,
    # points external to the line have a natural orientation parallel to the line (than can be use to rotate
    # texts or symbols to be aligned with the line.)
    def angle_at(parallel_distance, rotation=0)
      i,l = segment_position_of(parallel_distance)
      compute{segment(i).angle + rotation}
    end

    # multiply by matrix [[a11, a12], [a21, a22]]
    def transform(*t)
      LineString.new(*@vertices.map{|v| v.transform(*t)})
    end

    def apply(prc=nil, &blk)
      prc = prc || blk
      LineString.new(*@vertices.map{|v| prc[v]})
    end

    def contains?(point)
      self.locate_point(point)[1] == 0
    end

    def bounds
      xs = @vertices.map{|v| v.x}
      ys = @vertices.map{|v| v.y}
      [xs.min, ys.min, xs.max, ys.max]
    end

    private

    def segment_length(i)
      raise ArgumentError, "Invalid segment index #{i}" unless i>=0 && i<n_segments
      @segment_lengths ||= [nil]*n_segments
      @segment_lengths[i] ||= (@vertices[i+1]-@vertices[i]).modulus
    end

    def total_length
      l = Num(0)
      compute do
        (0...n_segments).each do |i|
          l += segment_length(i)
        end
      end
      l
    end

    # find segment and distance in segment corresponding to total parallel distance  TODO: rename
    def segment_position_of(l)
      i = 0
      max_i = n_segments-1
      compute do
        l = Num(l)
        while l>(s=segment_length(i)) && i<max_i
          l -= s
          i += 1
        end
      end
      return i, l
    end

    # compute parallel distance of position in segment TODO: rename
    def distance_along_line_of(segment_i, distance_in_segment)
      compute do
        l = Num(0)
        (0...segment_i).each do |i|
          l += segment_length(i)
        end
        l + distance_in_segment
      end
    end

  end # LineString

  def Point(*args)
    Vector(*args)
  end

  # Segment constructor
  def LineSegment(start_point, end_point)
    LineSegment.new(Vector(start_point), Vector(end_point))
  end

  # Line-string constructor
  def Line(*args)
    #if args.size<3
    #  LineSegment.new(*args.map{|arg| Vector(arg)})
    #else
      LineString.new(*args.map{|arg| Vector(arg)})
    #end
  end

  def rotation_transform(angle)
    Geo2D.context.num_context.math do
      sn = sin(angle)
      cs = cos(angle)
      [cs, -sn, sn, cs]
    end
  end

  # Rotation transformation; given the center of rotation (a point, i.e. a Vector) and the angle
  # this returns a procedure that can be used to apply the rotation to points.
  def rotation(center, angle)
    center = Vector(center)
    lambda{|p| Geo2D.context.num_context.eval{center + (p-center).rotate(angle)}}
  end

end