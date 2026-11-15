/* Reads a one-base, one-zone unstructured CGNS file with the CGNS library and prints what the
 * library saw, for tests/cgns.rs (SM_CONFORMANCE=1). Build: gcc cgns_dump.c -lcgns.
 *
 *   Z nverts ncells
 *   P x y z                      one per vertex
 *   S name type first last      then "C n v..." (connectivity) and "O n v..." (start offsets)
 *   B name n id...               BC point list; "L name location" follows
 *   R name n id...               ZoneSubRegion point list (cells)
 *   F name [bctype]              families under the base
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cgnslib.h>

#define CK(x)                                                              \
    do {                                                                   \
        if ((x)) {                                                         \
            fprintf(stderr, "cgns error at line %d: %s\n", __LINE__, cg_get_error()); \
            exit(2);                                                       \
        }                                                                  \
    } while (0)

static void need(int ok, const char *what) {
    if (!ok) {
        fprintf(stderr, "unexpected: %s\n", what);
        exit(3);
    }
}

static void ids(const char *tag, cgsize_t *v, cgsize_t n) {
    printf("%s %lld", tag, (long long)n);
    for (cgsize_t i = 0; i < n; i++) printf(" %lld", (long long)v[i]);
    printf("\n");
}

int main(int argc, char **argv) {
    int f, n, cd, pd;
    char name[64], bname[64];
    CK(cg_open(argv[1], CG_MODE_READ, &f));
    CK(cg_nbases(f, &n));
    need(n == 1, "one base");
    CK(cg_base_read(f, 1, bname, &cd, &pd));
    need(cd == 3 && pd == 3, "3-D base");
    CK(cg_nzones(f, 1, &n));
    need(n == 1, "one zone");
    CGNS_ENUMT(ZoneType_t) zt;
    CK(cg_zone_type(f, 1, 1, &zt));
    need(zt == CGNS_ENUMV(Unstructured), "unstructured zone");
    cgsize_t sz[3];
    CK(cg_zone_read(f, 1, 1, name, sz));
    printf("Z %lld %lld\n", (long long)sz[0], (long long)sz[1]);

    cgsize_t nv = sz[0], lo = 1, hi = sz[0];
    double *c[3];
    const char *cn[3] = {"CoordinateX", "CoordinateY", "CoordinateZ"};
    for (int k = 0; k < 3; k++) {
        c[k] = malloc(sizeof(double) * (nv ? nv : 1));
        CK(cg_coord_read(f, 1, 1, cn[k], CGNS_ENUMV(RealDouble), &lo, &hi, c[k]));
    }
    for (cgsize_t i = 0; i < nv; i++) printf("P %.17g %.17g %.17g\n", c[0][i], c[1][i], c[2][i]);

    int ns;
    CK(cg_nsections(f, 1, 1, &ns));
    for (int s = 1; s <= ns; s++) {
        CGNS_ENUMT(ElementType_t) t;
        cgsize_t first, last, size;
        int nbndry, parent;
        CK(cg_section_read(f, 1, 1, s, name, &t, &first, &last, &nbndry, &parent));
        CK(cg_ElementDataSize(f, 1, 1, s, &size));
        cgsize_t *conn = malloc(sizeof(cgsize_t) * (size ? size : 1));
        cgsize_t no = 0, *offs = NULL;
        if (t == CGNS_ENUMV(NGON_n) || t == CGNS_ENUMV(NFACE_n)) {
            no = last - first + 2;
            offs = malloc(sizeof(cgsize_t) * no);
            CK(cg_poly_elements_read(f, 1, 1, s, conn, offs, NULL));
        } else {
            CK(cg_elements_read(f, 1, 1, s, conn, NULL));
        }
        printf("S %s %d %lld %lld\n", name, (int)t, (long long)first, (long long)last);
        ids("C", conn, size);
        ids("O", offs, no);
        free(conn);
        free(offs);
    }

    int nb;
    CK(cg_nbocos(f, 1, 1, &nb));
    for (int b = 1; b <= nb; b++) {
        CGNS_ENUMT(BCType_t) bt;
        CGNS_ENUMT(PointSetType_t) pst;
        CGNS_ENUMT(DataType_t) dt;
        cgsize_t np, nls;
        int ni[3], nds;
        CK(cg_boco_info(f, 1, 1, b, name, &bt, &pst, &np, ni, &nls, &dt, &nds));
        need(pst == CGNS_ENUMV(PointList), "BC point list");
        cgsize_t *pl = malloc(sizeof(cgsize_t) * (np ? np : 1));
        CK(cg_boco_read(f, 1, 1, b, pl, NULL));
        printf("B %s", name);
        printf(" %lld", (long long)np);
        for (cgsize_t i = 0; i < np; i++) printf(" %lld", (long long)pl[i]);
        printf("\n");
        CGNS_ENUMT(GridLocation_t) loc;
        CK(cg_boco_gridlocation_read(f, 1, 1, b, &loc));
        printf("L %s %s\n", name, cg_GridLocationName(loc));
        free(pl);
    }

    int nr;
    CK(cg_nsubregs(f, 1, 1, &nr));
    for (int r = 1; r <= nr; r++) {
        int dim, bl, gl;
        CGNS_ENUMT(GridLocation_t) loc;
        CGNS_ENUMT(PointSetType_t) pst;
        cgsize_t np;
        CK(cg_subreg_info(f, 1, 1, r, name, &dim, &loc, &pst, &np, &bl, &gl));
        need(dim == 3 && loc == CGNS_ENUMV(CellCenter) && pst == CGNS_ENUMV(PointList), "cell sub-region");
        cgsize_t *pl = malloc(sizeof(cgsize_t) * (np ? np : 1));
        CK(cg_subreg_ptset_read(f, 1, 1, r, pl));
        printf("R %s", name);
        printf(" %lld", (long long)np);
        for (cgsize_t i = 0; i < np; i++) printf(" %lld", (long long)pl[i]);
        printf("\n");
        free(pl);
    }

    int nf;
    CK(cg_nfamilies(f, 1, &nf));
    for (int i = 1; i <= nf; i++) {
        int nfb, ng;
        CK(cg_family_read(f, 1, i, name, &nfb, &ng));
        if (nfb > 0) {
            char fb[64];
            CGNS_ENUMT(BCType_t) bt;
            CK(cg_fambc_read(f, 1, i, 1, fb, &bt));
            printf("F %s %s\n", name, cg_BCTypeName(bt));
        } else {
            printf("F %s\n", name);
        }
    }
    CK(cg_close(f));
    return 0;
}
