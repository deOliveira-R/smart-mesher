//! Paged CSR row arena with in-place edits.
//!
//! One flat `Vec<T>` holds every row; a row is a span `(start, len, cap)` into it. Rows grow by
//! doubling inside power-of-two blocks, so `splice` is a `copy_within` in the common case and a
//! relocation (amortized O(1)) when the block is full. Blocks released by retired cells go to
//! per-class free lists and are reused LIFO, so the layout depends only on the edit sequence
//! (deterministic) and long split-heavy sessions do not leak.

use sm_topo_api::TopoError;

/// Location of one row inside [`Rows::data`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Span {
    start: u32,
    len: u32,
    cap: u32,
}

/// Smallest block class: capacity `1 << MIN_CLASS`.
const MIN_CLASS: u32 = 2;

fn class_of(n: usize) -> u32 {
    let n = n.max(1 << MIN_CLASS);
    usize::BITS - (n - 1).leading_zeros()
}

/// A growable collection of rows of `T`, addressed by dense slot.
#[derive(Clone, Debug)]
pub(crate) struct Rows<T> {
    span: Vec<Span>,
    data: Vec<T>,
    /// `free[c]` holds the starts of free blocks of capacity `1 << c`.
    free: Vec<Vec<u32>>,
    /// Value that pads fresh blocks.
    fill: T,
}

impl<T: Copy> Rows<T> {
    /// An empty arena; `fill` pads unused block space.
    pub(crate) fn new(fill: T) -> Self {
        Self { span: Vec::new(), data: Vec::new(), free: Vec::new(), fill }
    }

    /// Append an empty row.
    pub(crate) fn push_empty(&mut self) {
        self.span.push(Span::default());
    }

    /// Remove the last slot (its row must be empty); returns its block to the free list.
    pub(crate) fn pop_slot(&mut self) {
        if let Some(sp) = self.span.pop() {
            debug_assert_eq!(sp.len, 0);
            self.release_block(sp);
        }
    }

    /// The row at `slot` (empty for an unknown slot).
    #[inline]
    pub(crate) fn row(&self, slot: usize) -> &[T] {
        match self.span.get(slot) {
            Some(sp) => &self.data[sp.start as usize..(sp.start + sp.len) as usize],
            None => &[],
        }
    }

    fn release_block(&mut self, sp: Span) {
        if sp.cap == 0 {
            return;
        }
        let c = sp.cap.trailing_zeros() as usize;
        if self.free.len() <= c {
            self.free.resize_with(c + 1, Vec::new);
        }
        self.free[c].push(sp.start);
    }

    fn take_block(&mut self, class: u32) -> Result<u32, TopoError> {
        let c = class as usize;
        if let Some(start) = self.free.get_mut(c).and_then(Vec::pop) {
            return Ok(start);
        }
        let start = u32::try_from(self.data.len()).map_err(|_| TopoError::IdExhausted)?;
        let cap = 1usize << class;
        if self.data.len().checked_add(cap).is_none_or(|n| u32::try_from(n).is_err()) {
            return Err(TopoError::IdExhausted);
        }
        self.data.resize(self.data.len() + cap, self.fill);
        Ok(start)
    }

    /// Replace `row[pos .. pos + remove]` by `insert`. `pos + remove` must be within the row.
    pub(crate) fn splice(&mut self, slot: usize, pos: usize, remove: usize, insert: &[T]) -> Result<(), TopoError> {
        let sp = self.span[slot];
        let len = sp.len as usize;
        debug_assert!(pos + remove <= len);
        let new_len = len - remove + insert.len();
        if new_len > sp.cap as usize {
            let class = class_of(new_len);
            let start = self.take_block(class)? as usize;
            let old = sp.start as usize;
            // Copy the kept head and tail around the replaced middle.
            self.data.copy_within(old..old + pos, start);
            self.data[start + pos..start + pos + insert.len()].copy_from_slice(insert);
            self.data.copy_within(old + pos + remove..old + len, start + pos + insert.len());
            self.release_block(sp);
            self.span[slot] = Span { start: start as u32, len: new_len as u32, cap: 1 << class };
        } else {
            let base = sp.start as usize;
            if insert.len() != remove {
                self.data.copy_within(base + pos + remove..base + len, base + pos + insert.len());
            }
            self.data[base + pos..base + pos + insert.len()].copy_from_slice(insert);
            self.span[slot].len = new_len as u32;
        }
        Ok(())
    }

    /// Empty the row and give its block back (for retired cells).
    pub(crate) fn release_row(&mut self, slot: usize) {
        let sp = self.span[slot];
        self.release_block(sp);
        self.span[slot] = Span::default();
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splice_grows_shrinks_and_reuses_blocks() {
        let mut r: Rows<u64> = Rows::new(0);
        r.push_empty();
        r.push_empty();
        r.splice(0, 0, 0, &[1, 2, 3]).unwrap();
        r.splice(1, 0, 0, &[9]).unwrap();
        r.splice(0, 3, 0, &[4, 5, 6, 7, 8]).unwrap();
        assert_eq!(r.row(0), &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(r.row(1), &[9]);
        r.splice(0, 1, 2, &[]).unwrap();
        assert_eq!(r.row(0), &[1, 4, 5, 6, 7, 8]);
        r.splice(0, 2, 1, &[40, 41]).unwrap();
        assert_eq!(r.row(0), &[1, 4, 40, 41, 6, 7, 8]);
        let before = r.data.len();
        r.release_row(0);
        r.splice(0, 0, 0, &[7; 8]).unwrap();
        assert_eq!(r.data.len(), before, "released block is reused");
        assert_eq!(r.row(0), &[7; 8]);
        assert_eq!(r.row(1), &[9]);
    }

    #[test]
    fn random_splices_match_vec() {
        let mut r: Rows<u64> = Rows::new(0);
        let mut model: Vec<Vec<u64>> = vec![Vec::new(); 5];
        for _ in 0..5 {
            r.push_empty();
        }
        let mut x = 0x9e37_79b9_7f4a_7c15u64;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x
        };
        for step in 0..4000u64 {
            let slot = (next() % 5) as usize;
            let len = model[slot].len();
            let pos = (next() % (len as u64 + 1)) as usize;
            let remove = if len == pos { 0 } else { (next() % (len - pos) as u64 + 1) as usize };
            let ins: Vec<u64> = (0..next() % 6).map(|k| step * 10 + k).collect();
            r.splice(slot, pos, remove, &ins).unwrap();
            model[slot].splice(pos..pos + remove, ins);
            if step % 97 == 0 {
                r.release_row(slot);
                model[slot].clear();
            }
        }
        for (s, m) in model.iter().enumerate() {
            assert_eq!(r.row(s), m.as_slice());
        }
    }
}
