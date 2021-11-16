Merge "Segment1.step";
SetFactory("OpenCASCADE");
Rectangle(74) = {-14, -1.7, -72.4, 60, 60, 0};
//+
BooleanIntersection{ Volume{1}; Delete; }{ Surface{74}; Delete; }
//+
Show "*";
