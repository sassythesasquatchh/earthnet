from dash import Dash, html, dcc, Input, Output
import pandas as pd
import plotly.express as px

# import os,sys


filename = "earthnet_database.csv"
df = pd.read_csv(filename, parse_dates=["time"])
df["measure_value::varchar"] = pd.to_numeric(df["measure_value::varchar"])

pd.unique(df["Device"])
coord_data = []
for device in pd.unique(df["Device"]):

    lat = (
        df.where((df["measure_name"] == "Lat") & (df["Device"] == device))
        .dropna()
        .sort_values(by=["time"])
        .iloc[0]["measure_value::varchar"]
    )
    long = (
        df.where((df["measure_name"] == "Long") & (df["Device"] == device))
        .dropna()
        .sort_values(by=["time"])
        .iloc[0]["measure_value::varchar"]
    )
    device_coord = [device, lat, long]
    coord_data.append(device_coord)

coord_df = pd.DataFrame(coord_data, columns=["device", "lat", "long"])


px.set_mapbox_access_token(
    "pk.eyJ1Ijoic2Fzc3l0aGVzYXNxdWF0Y2giLCJhIjoiY2w2dXJqNmxwMTN5MDNpcTk2NWJxbGN4MyJ9.cAzVjQ1D_8g8FsPIZhoteg"
)
fig = px.scatter_mapbox(
    coord_df,
    lat=coord_df.lat,
    lon=coord_df.long,
    hover_name="device",
    custom_data=["device"],
    zoom=1,
)
fig.update_layout({"plot_bgcolor": "rgba(0,0,0,0)", "paper_bgcolor": "rgba(0,0,0,0)"})


external_stylesheets = ["https://codepen.io/chriddyp/pen/bWLwgP.css"]


app = Dash(__name__, external_stylesheets=external_stylesheets)

server = app.server
component_style = {"width": "100%", "margin-bottom": "0px", "text-align": "center"}
dropdown_style = {
    "width": "100%",
    "margin-bottom": "30px",
    "text-align": "center",
    "margin-right": "auto",
    "margin-left": "auto",
    # "padding-left": "50px",
    # "padding-right": "50px",
}
inner_div_style = {"margin": "auto", "padding": "20px", "box-sizing": "border-box"}
outer_div_style = {"width": "100%", "background-color": "#FDFBF9"}
dropdown_options = [
    {"label": parameter, "value": parameter}
    for parameter in ["Temperature", "Humidity", "PM10", "PM25"]
]


# dropdown_options = [
#     {"label": parameter, "value": parameter}
#     for parameter in df["measure_name"].unique()
# ]

app.layout = html.Div(
    [
        html.Div(
            [
                html.Div(
                    [
                        dcc.Graph(
                            figure=fig,
                            id="map",
                            hoverData={"points": [{"customdata": ["NYUAD"]}]},
                        )
                    ],
                    style=component_style,
                ),
                html.Div(
                    [
                        dcc.Dropdown(
                            options=dropdown_options,
                            id="parameter_choice",
                            value="Temperature",
                            style={"padding-left": "76px", "padding-right": "76px"},
                        ),
                    ],
                    style=dropdown_style,
                ),  # Set width to 100% for the dropdown
                html.Div(
                    [dcc.Graph(id="timeseries")],
                    style=component_style,
                ),
            ],
            style=inner_div_style,
        ),
    ],
    style=outer_div_style,
)


def create_time_series(dff, chosen_parameter, device_name):

    fig = px.scatter(
        dff,
        x="time",
        y="measure_value::varchar",
        labels={"time": "Time", "measure_value::varchar": chosen_parameter},
    )

    fig.update_traces(mode="lines+markers")

    fig.update_xaxes(showgrid=False)
    fig.update_layout({"paper_bgcolor": "rgba(0,0,0,0)"})
    fig.update_layout(
        title_text=device_name,
        title_x=0.5,  # Center the title
        title_font=dict(size=20, family="Arial"),  # Set the font size and family
    )

    return fig


@app.callback(
    Output("timeseries", "figure"),
    Input("map", "hoverData"),
    Input("parameter_choice", "value"),
)
def update_timeseries(hoverData, chosen_parameter):
    # print(hoverData)
    # print(hoverData['points'][0]['customdata'][0])
    dependent_variable = hoverData["points"][0]["customdata"][0]
    dff = df[df["Device"] == dependent_variable]
    dff = dff[dff["measure_name"] == chosen_parameter]
    # print(dff)

    return create_time_series(dff, chosen_parameter, device_name=dependent_variable)


if __name__ == "__main__":
    app.run_server(debug=True)
