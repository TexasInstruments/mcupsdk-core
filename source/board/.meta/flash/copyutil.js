// Use OS agnostic nodeJS module to do file copy
const fs = require("fs");

let srcFile = process.argv[2];
let dstFile = process.argv[3];

fs.copyFile(srcFile, dstFile, (err) => {
    if (err) {
        console.log("Error Found:", err);
    }
    else {
        // do nothing
    }
});
