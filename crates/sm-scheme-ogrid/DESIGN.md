# sm-scheme-ogrid

The O-grid surface scheme (W4 tasks HEX-OGRID, HEX-OGRID-K; see drafts/hexpin.md D1): a self-contained kernel with ogrid and polar modes on a disk, an annulus with a 4-sided loop, and a two-circle ring. It does not depend on `sm-decomp`. It is registered in `sm_server::standard_schemes` (the array has 9 crates) but, in this W4-0.2 skeleton, `register` adds no scheme and the auto lists are unchanged. HEX-OGRID adds the scheme and its surface-list entries.
