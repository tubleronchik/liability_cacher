{ nixpkgs ? import (builtins.fetchTarball https://github.com/airalab/airapkgs/archive/14682605dee431671acfcf84f4d6fa3a26dd0e3f.tar.gz)
, system ? builtins.currentSystem
}:

let
  pkgs = nixpkgs { inherit system; };

in rec {
  package = pkgs.callPackage ./default.nix { };
}

