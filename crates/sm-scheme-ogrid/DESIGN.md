# sm-scheme-ogrid

The O-grid surface scheme (W4 task HEX-OGRID): meshes a circular face as a central block plus concentric quad rings, using the block layouts of `sm-decomp`. It is registered in `sm_server::standard_schemes` (the array has 9 crates) but, in this W4-0.2 skeleton, `register` adds no scheme and the auto lists are unchanged. HEX-OGRID adds the scheme and its surface-list entries.
