function in_segment = in_segment(points,seg)
lowx = min(seg(1:2:end));
lowy = min(seg(2:2:end));
highx = max(seg(1:2:end));
highy = max(seg(2:2:end));
for i = 1:size(points,1)
    if points(i,1) < lowx || points(i,1) > highx || points(i,2) < lowy || points(i,2) > highy
        in_segment = false;
        return
    end
end
in_segment = true;
end