use anyhow::{anyhow, bail, Result};
use elf::{
    abi,
    endian::LittleEndian,
    file::{Class, FileHeader},
    segment::ProgramHeader,
    ElfBytes,
};
use std::{collections::BTreeMap, fs, ops::Range, path::Path};
use tracing::debug;

// TODO - move to common lib
const FLASH_BASE: u64 = 0x08020000;
const FLASH_SIZE: u64 = 384 * 1024;

fn flash_partition() -> Range<u64> {
    Range {
        start: FLASH_BASE,
        end: FLASH_BASE + FLASH_SIZE,
    }
}

pub fn load_elf<P: AsRef<Path>>(elf_path: P) -> Result<Vec<u8>> {
    let data = fs::read(elf_path)?;
    let elf = ElfBytes::<LittleEndian>::minimal_parse(&data)?;
    sanity_check_elf(&elf.ehdr)?;
    Ok(data)
}

pub fn sanity_check_elf(ehdr: &FileHeader<LittleEndian>) -> Result<()> {
    if ehdr.class != Class::ELF32 {
        bail!("Bad class");
    }
    if ehdr.e_machine != abi::EM_ARM {
        bail!("Bad e_machine");
    }
    if !flash_partition().contains(&ehdr.e_entry) {
        bail!("Bad e_entry 0x{:X}", ehdr.e_entry);
    }

    Ok(())
}

const SH_VECTOR_TABLE: &str = ".vector_table";
const SH_TEXT: &str = ".text";
const SH_RODATA: &str = ".rodata";
const SH_DATA: &str = ".data";

// TODO
// - better error handling/reporting
// - maybe add a dry-run or w/e and generate the bin
//   in CI, sanity check matches what arm-none-eabi-objcopy -O binary would do
pub fn elf2bin(elf: &ElfBytes<LittleEndian>) -> Result<Vec<u8>> {
    debug!("Converting ELF to bin");

    let vector_table_sh = elf
        .section_header_by_name(SH_VECTOR_TABLE)?
        .ok_or_else(|| anyhow!("Missing {SH_VECTOR_TABLE} section header"))?;
    let text_sh = elf
        .section_header_by_name(SH_TEXT)?
        .ok_or_else(|| anyhow!("Missing {SH_TEXT} section header"))?;
    let rodata_sh = elf
        .section_header_by_name(SH_RODATA)?
        .ok_or_else(|| anyhow!("Missing {SH_RODATA} section header"))?;
    let data_sh = elf
        .section_header_by_name(SH_DATA)?
        .ok_or_else(|| anyhow!("Missing {SH_DATA} section header"))?;

    if vector_table_sh.sh_addr != FLASH_BASE {
        bail!(
            "Bad {SH_VECTOR_TABLE} address 0x{:X}",
            vector_table_sh.sh_addr
        );
    }
    if !flash_partition().contains(&text_sh.sh_addr) {
        bail!("Bad {SH_TEXT} address 0x{:X}", text_sh.sh_addr);
    }
    if !flash_partition().contains(&rodata_sh.sh_addr) {
        bail!("Bad {SH_RODATA} address 0x{:X}", rodata_sh.sh_addr);
    }

    let offset_to_address_alignments: BTreeMap<u64, u64> =
        [&vector_table_sh, &text_sh, &rodata_sh, &data_sh]
            .iter()
            .map(|sh| (sh.sh_offset, sh.sh_addralign))
            .collect();

    let program_headers: Vec<ProgramHeader> = elf
        .segments()
        .ok_or_else(|| anyhow!("Missing program headers"))?
        .into_iter()
        .collect();

    let vector_table_ph = program_headers
        .iter()
        .find(|ph| ph.p_offset == vector_table_sh.sh_offset)
        .ok_or_else(|| anyhow!("Missing {SH_VECTOR_TABLE} program header"))?;
    if vector_table_ph.p_flags != abi::PF_R {
        bail!("Bad {SH_VECTOR_TABLE} program header flags");
    }

    let text_ph = program_headers
        .iter()
        .find(|ph| ph.p_offset == text_sh.sh_offset)
        .ok_or_else(|| anyhow!("Missing {SH_TEXT} program header"))?;
    if text_ph.p_flags != abi::PF_R | abi::PF_X {
        bail!("Bad {SH_TEXT} program header flags");
    }

    let rodata_ph = program_headers
        .iter()
        .find(|ph| ph.p_offset == rodata_sh.sh_offset)
        .ok_or_else(|| anyhow!("Missing {SH_RODATA} program header"))?;
    if rodata_ph.p_flags != abi::PF_R {
        bail!("Bad {SH_RODATA} program header flags");
    }

    let data_ph = program_headers
        .iter()
        .find(|ph| ph.p_offset == data_sh.sh_offset)
        .ok_or_else(|| anyhow!("Missing {SH_DATA} program header"))?;
    if data_ph.p_flags != abi::PF_R | abi::PF_W {
        bail!("Bad {SH_DATA} program header flags");
    }

    let mut combined_segment_data = Vec::new();
    let mut bin_offset = 0_u64;
    let segments_to_write = vec![vector_table_ph, text_ph, rodata_ph, data_ph];
    for ph in segments_to_write.into_iter() {
        let address_alignment = *offset_to_address_alignments
            .get(&ph.p_offset)
            .ok_or_else(|| anyhow!("Missing address alignment entry"))?;
        let bin_offset_ptr = bin_offset as u32 as *const u32;
        let bin_align_offset_in_words = bin_offset_ptr.align_offset(address_alignment as usize);

        debug!(
            "Adding segment to bin at 0x{:X} (+ {} align words), vaddr 0x{:X}, paddr 0x{:X}, memsz 0x{:X} ({})",
            bin_offset, bin_align_offset_in_words, ph.p_vaddr, ph.p_paddr, ph.p_memsz, ph.p_memsz
        );

        if ph.p_type != abi::PT_LOAD {
            bail!("Bad program header type");
        }

        if ph.p_filesz != ph.p_memsz {
            bail!("FLASH segment sizes should match what's in the file");
        }

        if !flash_partition().contains(&ph.p_paddr) {
            bail!("Bad p_paddr 0x{:X}", ph.p_paddr);
        }

        let seg_data = elf.segment_data(ph)?;

        // Start with padding zero bytes to align the address
        for _ in 0..bin_align_offset_in_words {
            combined_segment_data.extend_from_slice(0_u32.to_le_bytes().as_slice());
            bin_offset += 4;
        }

        combined_segment_data.extend_from_slice(seg_data);

        bin_offset += ph.p_memsz;
    }

    if combined_segment_data.len() > FLASH_SIZE as usize {
        bail!(
            "Binary size {} exceeds the FLASH partition size",
            combined_segment_data.len()
        );
    }

    Ok(combined_segment_data)
}
