# Design notes:

I don't understand the need to convert between AngularRate/AngleCoordinate and Coordinate. Why does this conversion exist?

I don't see a reason for including `pitchRaw()`, `rollRaw()` and `yawRaw()`. just return the normalized values. they will be more readable.

`AngularCoordinate` to `AngularRate` and back does not make sense as a conversion. Remove.

I would like to try to incorporate the removal of `EulerAngles` here, where `AngularCoordinate` essentially takes its place everywhere. The reason for this is that there will be distinct patterns in the testing that should be detected. Rather than build a bunch of testing in parallel to `EulerAngles` as it exists, let's modify what currently exists and ensure it works as intended.