{
  description = "PINN Interference Manipulator: 1000Hz C++ deterministic controller ~50Hz JAX PINN inference.";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    instance-sim-engine.url = "github:elichall/instance-sim-engine";
  };

  outputs =
    {
      self,
      nixpkgs,
      instance-sim-engine,
    }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        buildInputs = with pkgs; [
          eigen
          boost
          rt
          instance-sim-engine.packages.${system}.engine
        ];
        nativeBuildInputs = with pkgs; [
          cmake
          clang-tools
          gcc
          basedpyright
          uv
        ];
        env = {
          # so uv python manager works
          NIX_LD = pkgs.stdenv.cc.bintools.dynamicLinker;
          NIX_LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
            pkgs.stdenv.cc.cc.lib
            pkgs.zlib
            pkgs.libffi
            pkgs.openssl
          ];
        };
        shellHook = ''
          # declair the paths for the shm brigdes as vars
          export TELEMETRY_SHM_DIR="/bin/shm/pinn_telemetry"
          export PATH_SHM_DIR="bin/shm/pinn_path"
        '';
      };
    };
}
