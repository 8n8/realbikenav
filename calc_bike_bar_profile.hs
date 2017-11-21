points :: [(Float, Float)]
points = [
    (6.66, 44.54),
    (8.74, 44.02),
    (10.16, 43.26),
    (18.52, 36.48),
    (20.3, 35.24),
    (21.5, 33.78),
    (25.12, 29.12),
    (29.52, 22.92),
    (33.82, 17.6),
    (37.72, 12.08),
    (41.7, 6.96),
    (43.46, 3.02)]

q :: Float
q = 43.52

calculateXandY :: (Float, Float) -> (Float, Float)
calculateXandY (a, b) =
    ((-q**4 + b**2 * (2*q**2 + 2*a**2) + 2*a**2*q**2 - b**4 - a**4)**0.5
     / (2*q),
     (q**2 + b**2 - a**2) / (2*q))

main :: IO()
main = print $ calculateXandY <$> points
