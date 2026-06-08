{...}: {
  projectRootFile = "flake.nix";
  programs.rustfmt.enable = true;
  programs.typstyle = {
    enable = true;
    lineWidth = 120;
  };
}
