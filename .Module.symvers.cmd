cmd_/home/zml/workspace/qemu/pci-echodev-zml/Module.symvers :=  sed 's/ko$$/o/'  /home/zml/workspace/qemu/pci-echodev-zml/modules.order | scripts/mod/modpost      -o /home/zml/workspace/qemu/pci-echodev-zml/Module.symvers -e -i Module.symvers -T - 
