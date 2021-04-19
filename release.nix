{ nixpkgs ? import (builtins.fetchTarball https://github.com/airalab/airapkgs/archive/9892c3d0901d29419f6b19e8f7bc175169e76b5b.tar.gz)
, system ? builtins.currentSystem
}:

let
  pkgs = nixpkgs { inherit system; };

in rec {
  package = pkgs.callPackage ./default.nix { };
}

