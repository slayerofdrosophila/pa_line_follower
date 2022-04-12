# HOW TO RUN

`roslaunch pa_line_follower pa_follower.launch`

# ADDITIONAL INFO

* The source file this was created from has a comment that says "hope for the best. lots of failure modes". However I have failed to find any failure modes myself besides things like "not seeing the color" which really isn't a failure

* The robot only looks at the bottom 20 pixels of the image when determining where to go. This is great for when the color may be present on the walls also.

* The "centroid" is computed by 