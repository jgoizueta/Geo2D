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
      other = Vector(other)
      Vector.new(self.x+other.x, self.y+other.y)
    end
    
    def -(other)
      other = Vector(other)
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
    
    # Apply arbitrary transformation (passed as a Proc or as a block)
    def apply(prc, &blk)
      prc ||= blk
      prc[self]
    end
    
    def coerce(scalar)
      if scalar.kind_of?(Numeric)
        [self, scalar]
      else
        raise ArgumentError, "Vector: cannot coerce #{scalar.class}"
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
        l,d = self.locate_point(point)
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
    # distance from the point to the line).
    # If the last parameter is true, the resulting point is forced to lie in the segment (so the distance along
    # the line is between 0 and the segment's length) and the second result is the distance from the point to the
    # segment (i.e. to the closest end of the segment if the projected point lies out of the segmen)
    def locate_point(point, corrected=false)  
      point = Vector(point)
      v = point - @start
      l = v.dot(direction)
      d = direction.cross_z(v) # == (v-l*direction).length == v.length*Math.sin(v.angle_to(direction))
      
      if corrected
        if l<0
          l = 0
          d = (point-@start).length
        elsif l>total_l
          l = self.length
          d = (point-@end).length
        end
      end
    
      [l, d]
    end
    
    # Computes the position of a point in the line given the distance along the line from the starting node.
    # If a second parameter is passed it indicates the separation of the computed point in the direction
    # perpendicular to the line; the point is on the left side of the line if the separation is > 0.
    def interpolate_point(parallel_distance, separation=0)
      p = @start + self.direction*parallel_distance
      p += direction.ortho*separation unless separation==0
      p
    end

    # Distance from the segment to a point
    def distance_to(point)
      locate_point(point, true).last
    end
    
    # Distance from the line that contains the segment to the point
    def line_distance_to(point)
      locate_point(point, false).last
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
      locate_point(point, true).last
    end
    
    def length_to(point)
      locate_point(point, true).first
    end
    
    # return parallalel distance and separation; 
    # if corrected, then parallalel distance is in [0,length] (the point is inside the line)
    # parallel distance in [0,length] , separation
    def locate_point(point, corrected=false)  
      best = nil
      
      total_l = 0
      (0...n_segments).each do |i|
        
        seg = segment(i)
        seg_l = seg.length
        
        l,d = seg.locate_point(point, false)
        max_i = n_segments-1
        
        if (l>0 || i==0) && (l<=seg_l || i==max_i)
          if best.nil? || d<best.last
            best = [total_l+l, d]
          end
        end  
        
        total_l += seg_l
      end
      
      if best && corrected
        l, d = best
        if l<0
          l = 0
          d = (point-points.first).length
        elsif l>total_l
          l = total_l
          d = (point-points.last).length
        end
        best = [l, d]
      end
      
      best
      
    end
    
    def interpolate_point(parallel_distance, separation=0, sweep=nil)
      # separation>0 => left side of line in direction of travel
      i, l = segment_position_of(parallel_distance)
      if sweep && separation!=0
        sweep = 0.0 unless sweep.kind_of?(Numeric)
        if i>0 && l<sweep
          a = 0.5*(segment(i-1).angle+segment(i).angle) + Math::PI/2
          @vertices[i] + separation*Vector(Math.cos(a), Math.sin(a))
        elsif i<(n_segments-1) && l>=(segment_length(i)-sweep)
          a = 0.5*(segment(i).angle+segment(i+1).angle) + Math::PI/2
          @vertices[i+1] + separation*Vector(Math.cos(a), Math.sin(a))
        else
          segment(i).interpolate_point(l, separation)
        end        
      else        
        segment(i).interpolate_point(l, separation)
      end
    end
    
    def angle_at(parallel_distance, sweep=false)
      i,l = segment_position_of(parallel_distance)
      if sweep
        sweep = 0.0 unless sweep.kind_of?(Numeric)
        if i>0 && l<sweep
          0.5*(segment(i-1).angle+segment(i).angle)
        elsif i<(n_segments-1) && l>=(segment_length(i)-sweep)
          0.5*(segment(i).angle+segment(i+1).angle)
        else
          segment(i).angle
        end        
      else
         segment(i).angle
      end
    end
    
    # multiply by matrix [[a11, a12], [a21, a22]]
    def transform(*t)
      LineString.new(*@vertices.map{|v| v.transforme(*t)})
    end
    
    def apply(prc=nil, &blk)
      prc = prc || blk
      LineString.new(*@vertices.map{|v| prc[v]})
    end
    
    def contains?(point)
      self.locate_point(point, true).last == 0
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
  
  # Rotation transformation; given the center of rotation (a point, i.e. a Vector) and the angle
  # this returns a procedure that can be used to apply the rotation to points.
  def rotation(center, angle)
    center = Vector(center)
    sn = Math.sin(angle)
    cs = Math.cos(angle)
    lambda{|p| center + (p-center).transform(cs, sn, -sn, cs)}
  end
  
end