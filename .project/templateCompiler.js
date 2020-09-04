let _ = require("lodash");

// Returns the compiled source from the template source.  The compiled source
// is in the form of a exports = templateFunction(argsObj); so it can be used
// just like a script would.

function compileTemplate(templateSource) {

    templateSource = adaptSource(templateSource);

    let compiled;
    try {
        compiled = _.template(templateSource, templateSettings).source;
    } catch (err) {
        if (!err.source || !_.startsWith(err.toString(), "SyntaxError")) {
            throw err;
        }
        compiled = err;
    }
    compiled = "return " + compiled + " ;"
    return compiled;
}

const templateSettings = {

    // Setup _.template to work just like RTSC/XDC
    // %%{\n
    // stmt ...\n
    // stmt ...\n
    // %%}\n
    // Multiple lines of script to be evaluated at this point.

    // Some notes about the regex:
    // %%\{ matches %%{
    // ([\s\S]*?) captures whatever is next as few times as possible
    // %%\} matches %%}
    // \r?\n? matches a eol on any OS if there, this way the block doesn't
    // include a newline in the output

    evaluate: /%%\{([\s\S]*?)%%\}\r?\n?/g,

    // text`script`text ... \n
    // A line of text to be output by the template at this
    // point. If present, all script between and including the
    // `` quotes is replaced with its (textual) value in the
    // current context.

    interpolate: /`([\s\S]*?)`/g,

    // This prevents lodash from generating a "with" statement to make the keys
    // of the passed in object instantiated.  Instead, "args" is the name of the
    // object passed in

    variable: "args",
};

function adaptSource(templateSource) {

    // % script ... \n
    // a single line of script evaluated by the template at
    // this point

    // Now, regex replace "% ..." with "%%{ ... %%}"
    // Some notes about the regex:
    // ^ matches a new line
    // [\t\f ] matches any tab, form feed or space
    //      we use this because \s would match \r\n
    // % matches %
    // (?!%) excludes matching if a % follows the first $ (to exclude %%{ and %%})
    // (.*?) matches and captures everything except the line terminator
    // $ matches the line terminator

    templateSource = templateSource.replace(/^[\t\f ]*%(?!%)(.*?)$/gm, "%%{$1\n%%}");

    // Next, if we have adjacent escaped blocks (ie adjacent %%} and %%{),
    // we shoudl combine them into one block.  This allows people to escape
    // adjacent lines using % but still have it be logically inline.  If we
    // don't do this, then _ will add a ; between the blocks.

    templateSource = templateSource.replace(/%%}\r?\n%%{/g, "");

    return templateSource;
}

function executeTemplate(templateSource, args) {
    let compile = compileTemplate(templateSource);
    let func1 = new Function(compile);
    let func2 = func1();

    return func2(args);
}

module.exports = {
    executeTemplate,
}
