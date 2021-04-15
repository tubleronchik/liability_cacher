{ nixpkgs ? import (builtins.fetchTarball https://github.com/tubleronchik/airapkgs/archive/7975139076e09ee9d8675acc2a23af7ec4ff7da6.tar.gz)
, system ? builtins.currentSystem
}:

let
  pkgs = nixpkgs { inherit system; };

in rec {
  package = pkgs.callPackage ./default.nix { };
}