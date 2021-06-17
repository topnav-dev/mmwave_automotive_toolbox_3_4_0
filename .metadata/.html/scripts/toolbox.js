var arrayToTable = function (data, attrs) {
    var table = $('<table />'),
        thead,
        tbody,
        tfoot,
        rows = [],
        row,
        link,
        i,
        j;

    table.attr(attrs);

    table.addClass("ui celled table");

    // loop through all the rows, we will deal with tfoot and thead later
    for (i = 0; i < data.length; i = i + 1) {
        row = $('<tr />');
        link = data[i].pop();
        //row.on("click", load_link(link));
        if (i != 0) {
          row.attr('onclick', 'load_link("'+link+'");');
        }

        for (j = 0; j < data[i].length; j = j + 1) {
            if (i === 0) {
                row.append($('<th />').html(data[i][j]));
            } else {
                row.append($('<td />').html(data[i][j]));
            }
        }
        rows.push(row);
    }

    // if we want a thead use shift to get it
    thead = rows.shift();
    thead = $('<thead />').append(thead);
    table.append(thead);

    tbody = $('<tbody />');

    // add all the rows
    for (i = 0; i < rows.length; i = i + 1) {
        tbody.append(rows[i]);
    }

    table.append(tbody);

    return table;
};

var insert_labs_table = function (tableID) {
  $.getJSON("data/labs.json", function( tableData ) {
    tableData.map(function(row){
      row[3] = "Labs/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTable(tableData, {}));
  });
};

var insert_experiments_table = function (tableID) {
  $.getJSON("data/experiments.json", function( tableData ) {
    tableData.map(function(row){
      row[2] = "Experiments/" + row[0];
      return row;
    });
    $(tableID).append(arrayToTable(tableData, {}));
  });
};

var insert_chirps_table = function (tableID) {
  $.getJSON("./chirps/chirpdb.json", function( tableData ) {
    $(tableID).append(arrayToTable(tableData, {}));
  });
};

var load_link = function(path) {
  console.log("Loading link " + path);
  path = path.replace(/\(/, "\\(");
  path = path.replace(/\)/, "\\)");
  path = path.split('/');
  var tree = parent.$('#jstree').jstree(true);

  var id;

  $.each(tree.get_json(), function(i, v) {
    if (v.text.search(new RegExp(/software/i)) != -1) {
      id = v.id;
      return;
    }
  });
  tree.open_node(id, function(node, status){
    $.each(tree.get_json(node).children, function(i, v) {
      if (v.text.search(new RegExp(/mmwave sensors/i)) != -1) {
        id = v.id;
        return;
      }
    });
    tree.open_node(id, function(node, status){
      $.each(tree.get_json(node).children, function(i, v) {
        if (v.text.search(new RegExp(/industrial toolbox/i)) != -1) {
          id = v.id;
          return;
        }
      });
      tree.open_node(id, function(node, status){
        $.each(tree.get_json(node).children, function(i, v) {
          if (v.text.search(new RegExp(path[0],"i")) != -1) {
            id = v.id;
            return;
          }
        });
        tree.open_node(id, function(node, status){
          console.log(path[1]);
          $.each(tree.get_json(node).children, function(i, v) {
            if (v.text.search(new RegExp(path[1],"i")) != -1) {
              tree.select_node(v.id);
              tree.open_node(v.id);
              return;
            }
          });
        }, false);
      }, false);
    }, false);
  }, false);
}
