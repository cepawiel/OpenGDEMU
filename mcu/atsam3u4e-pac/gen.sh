#!/bin/bash

set -e

svdtools patch svd/patch.yml svd/SAM3U4E_patched.svd
svdtools htmlcompare html svd/SAM3U4E.svd svd/SAM3U4E_patched.svd

svd2rust -i svd/SAM3U4E_patched.svd
rm -rf src
form -i lib.rs -o src/
rm lib.rs
cargo fmt
