use std::fs::File;
use std::io::{Read, BufReader};
use std::path::Path;

use xml::ParserConfig;
use xml::reader::{EventReader, XmlEvent};

use super::CalibDb;

fn consume_start<R: Read>(parser: &mut EventReader<R>, node_name: &str)
    -> Result<(), &'static str>
{
    match parser.next() {
        Ok(XmlEvent::StartElement { ref name, .. })
            if name.local_name == node_name => Ok(()),
        _ => return Err("Expected node start"),
    }
}

fn consume_end<R: Read>(parser: &mut EventReader<R>, node_name: &str)
    -> Result<(), &'static str>
{
    match parser.next() {
        Ok(XmlEvent::EndElement{ref name, .. })
            if name.local_name == node_name => Ok(()),
        _ => return Err("Expected node end"),
    }
}

fn get_node_val<R: Read>(parser: &mut EventReader<R>, node_name: &str)
    -> Result<String, &'static str>
{
    consume_start(parser, node_name)?;

    let val = if let Ok(XmlEvent::Characters(val)) = parser.next() {
        val
    } else {
        return Err("Expected characters")
    };

    consume_end(parser, node_name)?;
    Ok(val)
}

/*
fn parse_colors<R: Read>(parser: &mut EventReader<R>, db: &mut CalibDb)
    -> Result<(), &'static str>
{
    if get_node_val(parser, "count")? != "64" {
        return Err("count not equal to 64");
    }
    if get_node_val(parser, "item_version")? != "0" {
        return Err("item_version not equal to 0");
    }

    for i in 0..64 {
        consume_start(parser, "item")?;
        consume_start(parser, "rgb")?;

        if get_node_val(parser, "count")? != "3" {
            return Err("count not equal to 3");
        }

        db.lasers[i].color.0 = get_node_val(parser, "item")?.parse()
            .map_err(|_| "Failed to parse red color")?;
        db.lasers[i].color.1 = get_node_val(parser, "item")?.parse()
            .map_err(|_| "Failed to parse green color")?;
        db.lasers[i].color.2 = get_node_val(parser, "item")?.parse()
            .map_err(|_| "Failed to parse blue color")?;

        consume_end(parser, "rgb")?;
        consume_end(parser, "item")?;
    }
    Ok(())
}
*/

fn parse_min_intensity<R: Read>(parser: &mut EventReader<R>, db: &mut CalibDb)
    -> Result<(), &'static str>
{
    if get_node_val(parser, "count")? != "64" {
        return Err("count not equal to 64");
    }
    if get_node_val(parser, "item_version")? != "0" {
        return Err("item_version not equal to 0");
    }

    for i in 0..64 {
        let val = get_node_val(parser, "item")?;
        db.lasers[i].min_intensity = val.parse()
            .map_err(|_| "Failed to parse min_intensity")?;
    }
    Ok(())
}

fn parse_max_intensity<R: Read>(parser: &mut EventReader<R>, db: &mut CalibDb)
    -> Result<(), &'static str>
{
    if get_node_val(parser, "count")? != "64" {
        return Err("count not equal to 64");
    }
    if get_node_val(parser, "item_version")? != "0" {
        return Err("item_version not equal to 0");
    }

    for i in 0..64 {
        let val = get_node_val(parser, "item")?;
        db.lasers[i].max_intensity = val.parse()
            .map_err(|_| "Failed to parse max_intensity")?;
    }
    Ok(())
}

fn parse_point_item<R: Read>(parser: &mut EventReader<R>, db: &mut CalibDb)
    -> Result<(), &'static str>
{
    consume_start(parser, "item")?;
    consume_start(parser, "px")?;

    let i = get_node_val(parser, "id_")?.parse::<usize>()
        .map_err(|_| "Failed to parse i")?;

    let val: f32 = get_node_val(parser, "rotCorrection_")?
        .parse().map_err(|_| "Failed to parse rot_correction")?;
    let (sin, cos) = val.to_radians().sin_cos();
    db.lasers[i].rot_corr_sin = sin;
    db.lasers[i].rot_corr_cos = cos;

    let val: f32 = get_node_val(parser, "vertCorrection_")?
        .parse().map_err(|_| "Failed to parse vert_correction")?;
    let (sin, cos) = val.to_radians().sin_cos();
    db.lasers[i].rot_corr_sin = sin;
    db.lasers[i].rot_corr_cos = cos;

    db.lasers[i].dist_correction = get_node_val(parser, "distCorrection_")?
        .parse().map_err(|_| "Failed to parse dist_correction")?;
    db.lasers[i].dist_corr_x = get_node_val(parser, "distCorrectionX_")?
        .parse().map_err(|_| "Failed to parse dist_correction_x")?;
    db.lasers[i].dist_corr_y = get_node_val(parser, "distCorrectionY_")?
        .parse().map_err(|_| "Failed to parse dist_correction_y")?;
    db.lasers[i].vert_offset = get_node_val(parser, "vertOffsetCorrection_")?
        .parse().map_err(|_| "Failed to parse vert_offset_corr")?;
    db.lasers[i].horiz_offset = get_node_val(parser, "horizOffsetCorrection_")?
        .parse().map_err(|_| "Failed to parse horiz_offset_corr")?;
    db.lasers[i].focal_dist = get_node_val(parser, "focalDistance_")?
        .parse().map_err(|_| "Failed to parse focal_dist")?;
    db.lasers[i].focal_slope = get_node_val(parser, "focalSlope_")?
        .parse().map_err(|_| "Failed to parse focal_slope")?;

    consume_end(parser, "px")?;
    consume_end(parser, "item")?;
    Ok(())
}

fn parse_points<R: Read>(parser: &mut EventReader<R>, db: &mut CalibDb)
    -> Result<(), &'static str>
{
    if get_node_val(parser, "count")? != "64" {
        return Err("count not equal to 64");
    }
    if get_node_val(parser, "item_version")? != "1" {
        return Err("item_version not equal to 1");
    }

    for _ in 0..64 {
        parse_point_item(parser, db)?;
    }
    Ok(())
}

// TODO: replace error with io::Error

/// Read calibration XML file and parse data into `CalibDb` struct
pub fn read_db<P: AsRef<Path>>(path: P) -> Result<CalibDb, &'static str> {
    let file = File::open(path).map_err(|_| "DB file not found")?;
    let file = BufReader::new(file);

    let config = ParserConfig::new()
        .trim_whitespace(true);

    let mut parser = EventReader::new_with_config(file, config);
    let parser = &mut parser;

    let mut db = CalibDb::default();

    loop {
        let ev = parser.next();
        match ev {
            Ok(XmlEvent::StartElement { ref name, .. })
                if name.local_name == "DB" =>
            {
                db.dist_lsb = get_node_val(parser, "distLSB_")?
                    .parse().map_err(|_| "Failed to parse dist_lsb")?;;
            },
            /*
            Ok(XmlEvent::StartElement { ref name, .. })
                if name.local_name == "colors_" =>
            {
                parse_colors(parser, &mut db)?;
                consume_end(parser, "colors_")?;
            },
            */
            Ok(XmlEvent::StartElement { ref name, .. })
                if name.local_name == "minIntensity_" =>
            {
                parse_min_intensity(parser, &mut db)?;
                consume_end(parser, "minIntensity_")?;
            },
            Ok(XmlEvent::StartElement { ref name, .. })
                if name.local_name == "maxIntensity_" =>
            {
                parse_max_intensity(parser, &mut db)?;
                consume_end(parser, "maxIntensity_")?;
            },
            Ok(XmlEvent::StartElement { ref name, .. })
                if name.local_name == "points_" =>
            {
                parse_points(parser, &mut db)?;
                consume_end(parser, "points_")?;
            },
            Ok(XmlEvent::EndDocument) => break,
            Err(_) => return Err("Unexpected parsing error"),
            _ => (),
        }
    }

    Ok(db)
}
