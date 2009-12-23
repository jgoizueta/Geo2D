# Planar geometry of points and line-strings
module Geo2D

  # Planar vectors; used also to represent points of the plane
  class Vector

    def initialize(x=0, y=0)
      @x = x.to_f
      @y = y.to_f
    end

    attr_accessor :x, :y

    def modulus
      Math.hypot(self.x, self.y)
    end

    def length
      modulus
    end

    def argument
      Math.atan2(self.y, self.x)
    end

    def +(other)
      other = Geo2D.Vector(other)
      Vector.new(self.x+other.x, self.y+other.y)
    end

    def -(other)
      other = Geo2D.Vector(other)
      Vector.new(self.x-other.x, self.y-other.y)
    end

    def *(scalar_or_vector)
      if Numeric===scalar_or_vector
        # scalar product
        Vector.new(scalar_or_vector*self.x, scalar_or_vector*self.y)
      else
        # dot product
        other = Geo2D.Vector(scalar_or_vector)
        self.x*other.x +  self.y*other.y
      end
    end

    def /(scalar)
      # self * 1.0/scalar
      Vector.new(self.x/scalar, self.y/scalar)
    end

    # z coordinate of cross product
    def cross_z(other)
      self.x*other.y - other.x*self.y
    end

    def dot(other)
      self.x*other.x + self.y*other.y
    end

    def ==(other)
      self.x == other.x && self.y == other.y
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
      Math.atan2(cross_z(other), dot(other))
    end

    def aligned_with?(other)
      cross_z == 0
    end

    # multiply by matrix [[a11, a12], [a21, a22]]
    def transform(*t)
      a11, a12, a21, a22 = t.flatten
      x, y = self.x, self.y
      Vector.new(a11*x + a12*y, a21*x + a22*y)
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

    def distance_to(other)
      if other.kind_of?(Vector)
        (other-self).modulus
      else
        other.distance_to?(self)
      end
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
        # rotation = atan2(-(l.modulo(length))*d.sign, d.abs)        
        if l<0
          rotation = Math.atan2(d < 0 ? -l : l, d.abs)
          l = 0
          d = d/Math.cos(rotation) # d.sign*(point-@start).length
        elsif l>self.length
          l -= self.length
          rotation = Math.atan2(d < 0 ? -l : l, d.abs)
          l = self.length
          d = d/Math.cos(rotation) # d = d.sign*(point-@end).length
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

  end

  class LineString

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
    
    def <<(p)
      p = Geo2D.Point(*p)
      if @vertices.last != p
        @vertices << p
        @length = nil
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

      total_l = 0
      (0...n_segments).each do |i|

        seg = segment(i)
        seg_l = seg.length

        l,d,rotation = seg.locate_point(point, true)

        if best.nil? || d.abs<best[1].abs
          best = [total_l+l, d, rotation]
        end

        total_l += seg_l
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
      segment(i).angle + rotation
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
      l = 0
      (0...n_segments).each do |i|
        l += segment_length(i)
      end
      l
    end

    # find segment and distance in segment corresponding to total parallel distance  TODO: rename
    def segment_position_of(l)
      i = 0
      max_i = n_segments-1
      while l>(s=segment_length(i)) && i<max_i
        l -= s
        i += 1
      end
      return i, l
    end

    # compute parallel distance of position in segment TODO: rename
    def distance_along_line_of(segment_i, distance_in_segment)
      l = 0
      (0...segment_i).each do |i|
        l += segment_length(i)
      end
      l + distance_in_segment
    end

  end

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
    sn = Math.sin(angle)
    cs = Math.cos(angle)
    [cs, sn, -sn, cs]
  end

  # Rotation transformation; given the center of rotation (a point, i.e. a Vector) and the angle
  # this returns a procedure that can be used to apply the rotation to points.
  def rotation(center, angle)
    center = Vector(center)
    lambda{|p| center + (p-center).rotate(angle)}
  end

end