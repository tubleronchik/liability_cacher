{ nixpkgs ? import (builtins.fetchTarball https://github.com/airalab/airapkgs/archive/bd8deed59bfaf2b67496b2071876139013926976.tar.gz)
, system ? builtins.currentSystem
}:

let
  pkgs = nixpkgs { inherit system; };
in rec {
  package = pkgs.callPackage ./default.nix {  };
}
