-- Project-specific Neovim configuration

-- Set up YAML schema association for this project
vim.lsp.config('yamlls',
{
  settings = {
    yaml = {
      schemas = {
        ["https://jrl.cnrs.fr/mc_rtc/schemas/mc_rtc/mc_rtc.json"] = "**/mc_rtc.yaml",
        ["https://jrl.cnrs.fr/mc_rtc/schemas/mc_control/FSMController.json"] = "etc/MobileArmController.yaml",
        ["https://jrl.cnrs.fr/mc_rtc/schemas/mc_control/FSMStates.json"] = "src/states/data/*.yaml"
      },
      validate = true,
      format = { enable = false },
      hover = true,
      completion = true,
    }
  }
})
