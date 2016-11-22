# Modify the RUBYLIB environment variable to include the ignition math 
# library install path. For example, if you install to /user:
#
# $ export RUBYLIB=/usr/lib/ruby:$RUBYLIB
#
require 'ignition/math'

v1 = Ignition::Math::Vector3d.new(0, 0, 0)
printf("v =: %f %f %f\n", v1.X(), v1.Y(), v1.Z())

v2 = Ignition::Math::Vector3d.new(1, 0, 0)
printf("v2 = %f %f %f\n", v2.X(), v2.Y(), v2.Z())

v3 = v1 + v2
printf("v1 + v2 = %f %f %f\n", v3.X(), v3.Y(), v3.Z())

printf("v1.Distance(v2) = %f\n", v1.Distance(v2))