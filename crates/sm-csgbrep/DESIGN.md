# sm-csgbrep

CSG deck to B-rep import, checks and oracle (W4 tasks CSG-*): reads an OpenMC-style CSG deck and builds the B-rep model, with the checks and the oracle comparison that validate it. It is a layer-3 crate under the server and must not depend on `sm-server`. The `csgbrep` plugin is registered in `sm_server::standard()`; in this W4-0.2 skeleton it registers no op.
